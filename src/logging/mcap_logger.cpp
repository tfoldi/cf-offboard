#include "logging/mcap_logger.hpp"

#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <variant>

namespace cfo {

namespace {

std::uint64_t to_ns(std::chrono::system_clock::time_point tp) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            tp.time_since_epoch())
            .count());
}

const char* state_str(LinkState s) {
    switch (s) {
        case LinkState::Opened: return "opened";
        case LinkState::Closed: return "closed";
        case LinkState::Error:  return "error";
    }
    return "unknown";
}

std::string hex_payload(const std::uint8_t* data, std::size_t n) {
    static constexpr char hex[] = "0123456789abcdef";
    std::string out;
    out.resize(n * 2);
    for (std::size_t i = 0; i < n; ++i) {
        out[2 * i]     = hex[(data[i] >> 4) & 0x0F];
        out[2 * i + 1] = hex[data[i] & 0x0F];
    }
    return out;
}

void json_escape_into(std::string& out, std::string_view s) {
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
}

constexpr std::string_view kLinkSchema = R"({
  "type":"object",
  "properties":{
    "state":{"type":"string","enum":["opened","closed","error"]},
    "detail":{"type":"string"}
  },
  "required":["state","detail"]
})";

constexpr std::string_view kRawSchema = R"({
  "type":"object",
  "properties":{
    "port":{"type":"integer"},
    "channel":{"type":"integer"},
    "payload":{"type":"string","description":"hex-encoded CRTP payload bytes"}
  },
  "required":["port","channel","payload"]
})";

constexpr std::string_view kConsoleSchema = R"({
  "type":"object",
  "properties":{
    "text":{"type":"string","description":"firmware console fragment"}
  },
  "required":["text"]
})";

constexpr std::string_view kStateSchema = R"({
  "type":"object",
  "properties":{
    "fw_t_ms":{"type":"integer","description":"firmware monotonic clock, ms"},
    "x":{"type":"number"},"y":{"type":"number"},"z":{"type":"number"},
    "vbat":{"type":"number","description":"battery voltage, V"},
    "roll":{"type":"number","description":"deg"},
    "pitch":{"type":"number","description":"deg"},
    "yaw":{"type":"number","description":"deg"}
  },
  "required":["fw_t_ms","x","y","z","vbat","roll","pitch","yaw"]
})";

} // namespace

struct MCAPLogger::Impl {
    Impl(std::filesystem::path p, std::size_t cap)
        : path{std::move(p)}, capacity{cap} {}

    std::filesystem::path path;
    std::size_t capacity;
    std::atomic<std::uint64_t> dropped{0};

    std::mutex m;
    std::condition_variable cv;
    std::deque<LogEvent> queue;
    bool stop{false};
    bool closed{false};

    mcap::McapWriter writer;
    mcap::ChannelId link_channel_id{0};
    mcap::ChannelId raw_channel_id{0};
    mcap::ChannelId cmd_channel_id{0};
    mcap::ChannelId console_channel_id{0};
    mcap::ChannelId state_channel_id{0};
    std::uint32_t link_seq{0};
    std::uint32_t raw_seq{0};
    std::uint32_t cmd_seq{0};
    std::uint32_t console_seq{0};
    std::uint32_t state_seq{0};

    std::thread thread;

    void run() {
        std::deque<LogEvent> batch;
        for (;;) {
            {
                std::unique_lock lock{m};
                cv.wait(lock, [&] { return stop || !queue.empty(); });
                std::swap(batch, queue);
                if (batch.empty() && stop) return;
            }
            for (auto& ev : batch) {
                std::visit([&](auto& e) { write_event(e); }, ev);
            }
            batch.clear();
        }
    }

    void write_event(const LinkEvent& e) {
        std::string payload;
        payload.reserve(64 + e.detail.size());
        payload += R"({"state":")";
        payload += state_str(e.state);
        payload += R"(","detail":")";
        json_escape_into(payload, e.detail);
        payload += R"("})";
        emit(link_channel_id, ++link_seq, e.t, payload);
    }

    void write_event(const RawTelemetryEvent& e) {
        std::string payload = serialize_raw(e.packet);
        emit(raw_channel_id, ++raw_seq, e.t, payload);
    }

    void write_event(const RawCommandEvent& e) {
        std::string payload = serialize_raw(e.packet);
        emit(cmd_channel_id, ++cmd_seq, e.t, payload);
    }

    void write_event(const ConsoleEvent& e) {
        std::string payload;
        payload.reserve(16 + e.text.size());
        payload += R"({"text":")";
        json_escape_into(payload, e.text);
        payload += R"("})";
        emit(console_channel_id, ++console_seq, e.t, payload);
    }

    void write_event(const LogBlockEvent& e) {
        char buf[256];
        const auto& s = e.sample;
        const int n = std::snprintf(
            buf, sizeof(buf),
            R"({"fw_t_ms":%u,"x":%.6f,"y":%.6f,"z":%.6f,)"
            R"("vbat":%.4f,"roll":%.4f,"pitch":%.4f,"yaw":%.4f})",
            static_cast<unsigned>(s.timestamp_ms),
            s.x, s.y, s.z, s.vbat, s.roll, s.pitch, s.yaw);
        emit(state_channel_id, ++state_seq, e.t,
             std::string{buf, static_cast<std::size_t>(n)});
    }

    static std::string serialize_raw(const RawPacket& pkt) {
        std::string payload;
        payload.reserve(48 + 2 * pkt.size);
        payload += R"({"port":)";
        payload += std::to_string(pkt.port);
        payload += R"(,"channel":)";
        payload += std::to_string(pkt.channel);
        payload += R"(,"payload":")";
        payload += hex_payload(pkt.payload.data(), pkt.size);
        payload += R"("})";
        return payload;
    }

    void emit(mcap::ChannelId ch, std::uint32_t seq,
              std::chrono::system_clock::time_point t,
              const std::string& payload) {
        const auto ns = to_ns(t);
        mcap::Message msg;
        msg.channelId = ch;
        msg.sequence = seq;
        msg.logTime = ns;
        msg.publishTime = ns;
        msg.data = reinterpret_cast<const std::byte*>(payload.data());
        msg.dataSize = payload.size();
        (void)writer.write(msg);
    }
};

MCAPLogger::MCAPLogger(std::unique_ptr<Impl> impl) : impl_{std::move(impl)} {}

MCAPLogger::~MCAPLogger() {
    close();
}

std::expected<std::unique_ptr<MCAPLogger>, LoggerError>
MCAPLogger::create(std::filesystem::path path, std::size_t queue_capacity) {
    auto impl = std::make_unique<Impl>(std::move(path), queue_capacity);

    mcap::McapWriterOptions opts{"cf-offboard"};
    opts.compression = mcap::Compression::None;
    auto status = impl->writer.open(impl->path.string(), opts);
    if (!status.ok()) {
        return std::unexpected(LoggerError::OpenFailed);
    }

    mcap::Schema link_schema{"cfo.LinkEvent", "jsonschema", kLinkSchema};
    impl->writer.addSchema(link_schema);
    mcap::Channel link_channel{"/link", "json", link_schema.id};
    impl->writer.addChannel(link_channel);
    impl->link_channel_id = link_channel.id;

    mcap::Schema raw_schema{"cfo.RawPacket", "jsonschema", kRawSchema};
    impl->writer.addSchema(raw_schema);

    mcap::Channel raw_channel{"/telemetry/raw", "json", raw_schema.id};
    impl->writer.addChannel(raw_channel);
    impl->raw_channel_id = raw_channel.id;

    mcap::Channel cmd_channel{"/command/raw", "json", raw_schema.id};
    impl->writer.addChannel(cmd_channel);
    impl->cmd_channel_id = cmd_channel.id;

    mcap::Schema console_schema{"cfo.Console", "jsonschema", kConsoleSchema};
    impl->writer.addSchema(console_schema);
    mcap::Channel console_channel{"/telemetry/console", "json", console_schema.id};
    impl->writer.addChannel(console_channel);
    impl->console_channel_id = console_channel.id;

    mcap::Schema state_schema{"cfo.LogBlock", "jsonschema", kStateSchema};
    impl->writer.addSchema(state_schema);
    mcap::Channel state_channel{"/telemetry/state", "json", state_schema.id};
    impl->writer.addChannel(state_channel);
    impl->state_channel_id = state_channel.id;

    auto* p = impl.get();
    impl->thread = std::thread{[p] { p->run(); }};

    return std::unique_ptr<MCAPLogger>{new MCAPLogger{std::move(impl)}};
}

void MCAPLogger::log(LogEvent ev) {
    auto& s = *impl_;
    {
        std::lock_guard lock{s.m};
        if (s.closed) return;
        if (s.queue.size() >= s.capacity) {
            s.dropped.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        s.queue.push_back(std::move(ev));
    }
    s.cv.notify_one();
}

std::uint64_t MCAPLogger::dropped_count() const noexcept {
    return impl_ ? impl_->dropped.load(std::memory_order_relaxed) : 0;
}

void MCAPLogger::close() {
    if (!impl_) return;
    {
        std::lock_guard lock{impl_->m};
        if (impl_->closed) return;
        impl_->closed = true;
        impl_->stop = true;
    }
    impl_->cv.notify_all();
    if (impl_->thread.joinable()) impl_->thread.join();
    impl_->writer.close();
}

} // namespace cfo
