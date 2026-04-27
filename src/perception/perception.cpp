#include "perception/perception.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace cfo {

namespace {

bool in_range(std::uint16_t mm, const PerceptionConfig& cfg) noexcept {
    const float m = static_cast<float>(mm) * 0.001f;
    return m >= cfg.range_min_m && m <= cfg.range_max_m;
}

float to_metres(std::uint16_t mm) noexcept {
    return static_cast<float>(mm) * 0.001f;
}

} // namespace

RangeSnapshot make_range_snapshot(const RangeBlockSample& src,
                                  const PerceptionConfig& cfg,
                                  std::chrono::steady_clock::time_point t_recv,
                                  std::uint64_t update_count) noexcept {
    RangeSnapshot s{};
    s.fw_timestamp_ms = src.timestamp_ms;
    s.front_m = to_metres(src.front_mm);
    s.back_m  = to_metres(src.back_mm);
    s.left_m  = to_metres(src.left_mm);
    s.right_m = to_metres(src.right_mm);
    s.up_m    = to_metres(src.up_mm);
    s.valid_front = in_range(src.front_mm, cfg);
    s.valid_back  = in_range(src.back_mm,  cfg);
    s.valid_left  = in_range(src.left_mm,  cfg);
    s.valid_right = in_range(src.right_mm, cfg);
    s.valid_up    = in_range(src.up_mm,    cfg);
    s.t_recv = t_recv;
    s.update_count = update_count;
    return s;
}

std::array<BodyFrameRay, 5>
rays_from_snapshot(const RangeSnapshot& s) noexcept {
    auto make = [](RangeSensor id, float dx, float dy, float dz,
                   float r, bool valid) {
        BodyFrameRay ray{};
        ray.sensor = id;
        ray.dx = dx; ray.dy = dy; ray.dz = dz;
        ray.range_m = r;
        ray.valid = valid;
        return ray;
    };
    return {
        make(RangeSensor::Front, +1.0f, 0.0f, 0.0f, s.front_m, s.valid_front),
        make(RangeSensor::Back,  -1.0f, 0.0f, 0.0f, s.back_m,  s.valid_back),
        make(RangeSensor::Left,   0.0f, +1.0f, 0.0f, s.left_m, s.valid_left),
        make(RangeSensor::Right,  0.0f, -1.0f, 0.0f, s.right_m, s.valid_right),
        make(RangeSensor::Up,     0.0f, 0.0f, +1.0f, s.up_m,   s.valid_up),
    };
}

std::optional<ObstaclePoint>
project_to_odom(const BodyFrameRay& ray, const VehicleState& v,
                std::chrono::steady_clock::time_point now,
                const PerceptionConfig& cfg) noexcept {
    if (!ray.valid) return std::nullopt;
    if (v.update_count == 0) return std::nullopt;
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - v.last_update);
    if (age > cfg.max_pose_age) return std::nullopt;

    // Body-frame endpoint = origin + range * direction.
    const float bx = ray.origin_x + ray.range_m * ray.dx;
    const float by = ray.origin_y + ray.range_m * ray.dy;
    const float bz = ray.origin_z + ray.range_m * ray.dz;

    // Rotate the horizontal components into world frame using current
    // yaw (drone is level enough at hover for this to be accurate
    // within the perception-thresholds used by the mission logic).
    const float yaw = v.attitude.yaw;
    const float cy = std::cos(yaw), sy = std::sin(yaw);

    ObstaclePoint p{};
    p.sensor = ray.sensor;
    p.x = v.position.x + (bx * cy - by * sy);
    p.y = v.position.y + (bx * sy + by * cy);
    p.z = v.position.z + bz;
    p.t = now;
    return p;
}

ObstacleStatus classify_forward(const RangeSnapshot& s,
                                 const PerceptionConfig& cfg) noexcept {
    if (!s.valid_front) {
        // No reading inside [range_min, range_max] — i.e. nothing within
        // sensor range. Treat as Clear. (Other interpretations: "sensor
        // dead" → block; we err on the permissive side here because the
        // actual blocked condition is "I see something close".)
        return ObstacleStatus::Clear;
    }
    if (s.front_m < cfg.blocked_m) return ObstacleStatus::Blocked;
    if (s.front_m < cfg.caution_m) return ObstacleStatus::Caution;
    return ObstacleStatus::Clear;
}

void ObstacleStore::push(ObstaclePoint p) {
    std::lock_guard lock{m_};
    points_.push_back(p);
}

void ObstacleStore::prune(std::chrono::steady_clock::time_point now) {
    std::lock_guard lock{m_};
    const auto cutoff = now - cfg_.obstacle_lifetime;
    while (!points_.empty() && points_.front().t < cutoff) {
        points_.pop_front();
    }
}

std::vector<ObstaclePoint> ObstacleStore::snapshot() const {
    std::lock_guard lock{m_};
    return std::vector<ObstaclePoint>{points_.begin(), points_.end()};
}

std::size_t ObstacleStore::size() const noexcept {
    std::lock_guard lock{m_};
    return points_.size();
}

} // namespace cfo
