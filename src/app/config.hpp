#pragma once

#include <chrono>
#include <filesystem>
#include <string>

namespace cfo {

// Minimal application configuration. Hardcoded defaults today; loaded from
// CLI args / a config file later. Kept as a plain value type with explicit
// fields — not a settings framework.
struct AppConfig {
    std::string crazyflie_uri{"radio://0/80/2M/E7E7E7E7E7"};
    std::chrono::milliseconds rx_poll_timeout{50};
    std::filesystem::path mcap_path{"flight.mcap"};
};

} // namespace cfo
