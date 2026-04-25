// Offline MCAP inspector — summarizes /telemetry/raw traffic by (port, channel)
// so we can see what packet types the Crazyflie is actually sending before
// designing decoders. Single-purpose; not part of the runtime.
//
// Usage:
//   cfo_inspect <flight.mcap>

#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

struct Stats {
    std::uint64_t count = 0;
    std::uint64_t total_payload_bytes = 0;
    std::string sample_hex;  // first sample, hex-encoded, truncated
};

// We control the JSON encoding (see mcap_logger.cpp) so a flat scan is
// sufficient: keys appear once, no nesting, no escapes inside the values.

std::string_view extract_string(std::string_view json, std::string_view key) {
    std::string needle;
    needle.reserve(key.size() + 4);
    needle += '"';
    needle += key;
    needle += "\":\"";
    auto pos = json.find(needle);
    if (pos == std::string_view::npos) return {};
    pos += needle.size();
    auto end = json.find('"', pos);
    if (end == std::string_view::npos) return {};
    return json.substr(pos, end - pos);
}

int extract_int(std::string_view json, std::string_view key) {
    std::string needle;
    needle.reserve(key.size() + 3);
    needle += '"';
    needle += key;
    needle += "\":";
    auto pos = json.find(needle);
    if (pos == std::string_view::npos) return -1;
    pos += needle.size();
    int value = 0;
    while (pos < json.size() &&
           std::isdigit(static_cast<unsigned char>(json[pos]))) {
        value = value * 10 + (json[pos] - '0');
        ++pos;
    }
    return value;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <flight.mcap>\n", argv[0]);
        return 1;
    }

    mcap::McapReader reader;
    auto status = reader.open(argv[1]);
    if (!status.ok()) {
        std::fprintf(stderr, "open failed: %s\n", status.message.c_str());
        return 1;
    }

    std::map<std::pair<int, int>, Stats> hist;
    std::uint64_t total_raw = 0;
    std::uint64_t link_events = 0;

    auto messages = reader.readMessages();
    for (const auto& mv : messages) {
        if (!mv.channel) continue;
        std::string_view topic = mv.channel->topic;
        std::string_view payload{
            reinterpret_cast<const char*>(mv.message.data),
            mv.message.dataSize};

        if (topic == "/link") {
            std::printf("[link] %.*s\n",
                        static_cast<int>(payload.size()), payload.data());
            ++link_events;
        } else if (topic == "/telemetry/raw") {
            ++total_raw;
            int port = extract_int(payload, "port");
            int ch = extract_int(payload, "channel");
            auto hex = extract_string(payload, "payload");

            auto& s = hist[{port, ch}];
            ++s.count;
            s.total_payload_bytes += hex.size() / 2;
            if (s.sample_hex.empty()) {
                s.sample_hex = std::string{hex.substr(
                    0, std::min<std::size_t>(hex.size(), 32))};
            }
        }
    }
    reader.close();

    std::printf("\n--- raw packet histogram (%llu raw msgs, %llu link events) ---\n",
                static_cast<unsigned long long>(total_raw),
                static_cast<unsigned long long>(link_events));
    std::printf("%6s %4s %10s %8s   %s\n",
                "port", "ch", "count", "%total", "sample (first 16B hex)");

    std::vector<std::pair<std::pair<int, int>, const Stats*>> rows;
    rows.reserve(hist.size());
    for (const auto& [k, v] : hist) rows.emplace_back(k, &v);
    std::sort(rows.begin(), rows.end(),
              [](auto& a, auto& b) { return a.second->count > b.second->count; });

    for (const auto& [k, s] : rows) {
        const double pct = total_raw ? 100.0 * static_cast<double>(s->count) /
                                          static_cast<double>(total_raw)
                                     : 0.0;
        std::printf("%6d %4d %10llu %7.1f%%   %s\n",
                    k.first, k.second,
                    static_cast<unsigned long long>(s->count), pct,
                    s->sample_hex.c_str());
    }
    return 0;
}
