#pragma once

#include "logging/types.hpp"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <memory>

namespace cfo {

enum class LoggerError : std::uint8_t {
    OpenFailed,
};

// Asynchronous MCAP file logger.
//
// log() is non-blocking and may be called from any thread. Events go through
// a bounded in-memory queue; if the queue is full the event is dropped and
// the drop counter increments. Serialization and disk I/O run on a dedicated
// background thread, so the control loop never blocks on the logger.
//
// close() blocks until the queue is drained and the MCAP file is finalized.
// The destructor calls close() if the caller has not.
//
// MCAP-specific types (writer, schemas, channels) are confined to the .cpp.
class MCAPLogger {
public:
    static std::expected<std::unique_ptr<MCAPLogger>, LoggerError>
    create(std::filesystem::path path, std::size_t queue_capacity = 4096);

    ~MCAPLogger();

    MCAPLogger(const MCAPLogger&) = delete;
    MCAPLogger& operator=(const MCAPLogger&) = delete;

    void log(LogEvent ev);

    [[nodiscard]] std::uint64_t dropped_count() const noexcept;

    void close();

private:
    struct Impl;
    explicit MCAPLogger(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

} // namespace cfo
