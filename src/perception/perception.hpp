#pragma once

#include "crazyflie/protocol/types.hpp"
#include "perception/types.hpp"
#include "state/types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace cfo {

// Per-sensor hardware bounds. Multiranger is rated 0.05 m to ~4 m;
// readings outside that window are treated as invalid (no obstacle in
// view rather than "obstacle at +∞").
struct PerceptionConfig {
    float range_min_m{0.05f};
    float range_max_m{4.0f};

    // Pose freshness budget — projected points are only computed if the
    // vehicle's last_update is within this window.
    std::chrono::milliseconds max_pose_age{500};

    // Rolling obstacle store window — projected points expire after this.
    std::chrono::milliseconds obstacle_lifetime{2000};

    // Forward-obstacle thresholds (front_m). v1 uses two zones — anything
    // closer than `blocked_m` blocks, anything between caution and blocked
    // is caution; further is clear.
    float blocked_m{0.50f};
    float caution_m{1.00f};
};

// Pure: build a RangeSnapshot from a decoded firmware sample, applying
// the validity rules from PerceptionConfig.
[[nodiscard]] RangeSnapshot
make_range_snapshot(const RangeBlockSample& src,
                    const PerceptionConfig& cfg,
                    std::chrono::steady_clock::time_point t_recv,
                    std::uint64_t update_count) noexcept;

// Pure: convert a RangeSnapshot into the five canonical body-frame rays.
[[nodiscard]] std::array<BodyFrameRay, 5>
rays_from_snapshot(const RangeSnapshot& s) noexcept;

// Pure: project a body-frame ray's endpoint into `odom` using the
// vehicle's current position and yaw. Drones at hover are essentially
// level, so we use yaw-only rotation — sufficient for 4 m-scale obstacle
// detection. Returns nullopt if the ray is invalid or the pose is stale.
[[nodiscard]] std::optional<ObstaclePoint>
project_to_odom(const BodyFrameRay& ray,
                const VehicleState& v,
                std::chrono::steady_clock::time_point now,
                const PerceptionConfig& cfg) noexcept;

// Pure: classify the forward range into a coarse zone for mission logic.
[[nodiscard]] ObstacleStatus
classify_forward(const RangeSnapshot& s,
                 const PerceptionConfig& cfg) noexcept;

// Rolling buffer of recent ObstaclePoints in `odom`. Old points expire
// based on PerceptionConfig::obstacle_lifetime. Thread-safe (simple mutex).
class ObstacleStore {
public:
    explicit ObstacleStore(PerceptionConfig cfg = {}) : cfg_{cfg} {}

    void push(ObstaclePoint p);
    void prune(std::chrono::steady_clock::time_point now);

    // Snapshot of all live points, oldest first.
    [[nodiscard]] std::vector<ObstaclePoint> snapshot() const;
    [[nodiscard]] std::size_t size() const noexcept;

private:
    PerceptionConfig cfg_;
    mutable std::mutex m_;
    std::deque<ObstaclePoint> points_;
};

} // namespace cfo
