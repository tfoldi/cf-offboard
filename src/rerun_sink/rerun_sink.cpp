#include "rerun_sink/rerun_sink.hpp"

#include "perception/perception.hpp"

#include <rerun.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

namespace cfo {

struct RerunSink::Impl {
    rerun::RecordingStream rec;
    explicit Impl(std::string_view app_id) : rec{std::string{app_id}} {}
};

namespace {

const char* obstacle_status_short(ObstacleStatus s) noexcept {
    switch (s) {
        case ObstacleStatus::Clear:   return "clear";
        case ObstacleStatus::Caution: return "caution";
        case ObstacleStatus::Blocked: return "blocked";
    }
    return "?";
}

// Convert (roll, pitch, yaw) Euler angles in radians to a quaternion
// (x, y, z, w). ZYX convention (yaw → pitch → roll).
std::array<float, 4> euler_zyx_to_quat(float roll, float pitch, float yaw) noexcept {
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f),   sy = std::sin(yaw * 0.5f);
    return {
        sr * cp * cy - cr * sp * sy,   // x
        cr * sp * cy + sr * cp * sy,   // y
        cr * cp * sy - sr * sp * cy,   // z
        cr * cp * cy + sr * sp * sy,   // w
    };
}

const char* sensor_path(RangeSensor s) noexcept {
    switch (s) {
        case RangeSensor::Front: return "base_link/ranges/front";
        case RangeSensor::Back:  return "base_link/ranges/back";
        case RangeSensor::Left:  return "base_link/ranges/left";
        case RangeSensor::Right: return "base_link/ranges/right";
        case RangeSensor::Up:    return "base_link/ranges/up";
    }
    return "base_link/ranges/?";
}

} // namespace

RerunSink::RerunSink(std::unique_ptr<Impl> impl) : impl_{std::move(impl)} {}
RerunSink::~RerunSink() = default;

std::expected<std::unique_ptr<RerunSink>, std::string>
RerunSink::create(std::string_view app_id, Mode mode) {
    auto impl = std::make_unique<Impl>(app_id);

    rerun::Error err = (mode == Mode::Spawn) ? impl->rec.spawn()
                                              : impl->rec.connect_grpc();
    if (err.is_err()) {
        return std::unexpected(std::string{err.description});
    }

    // Static "world" axes — handy reference at the origin.
    impl->rec.log_static("odom",
        rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    return std::unique_ptr<RerunSink>{new RerunSink{std::move(impl)}};
}

void RerunSink::log_vehicle(const VehicleState& v) {
    const auto q = euler_zyx_to_quat(
        v.attitude.roll, v.attitude.pitch, v.attitude.yaw);

    impl_->rec.log("odom/vehicle",
        rerun::Transform3D::from_translation_rotation(
            {v.position.x, v.position.y, v.position.z},
            rerun::Quaternion::from_xyzw({q[0], q[1], q[2], q[3]})));

    // Cumulative path trail — Rerun keeps each Points3D entry in
    // timeline order; the viewer can show all of them as a trail.
    impl_->rec.log("odom/path_actual",
        rerun::Points3D({{v.position.x, v.position.y, v.position.z}})
            .with_radii({0.01f}));
}

void RerunSink::log_battery(float volts) {
    impl_->rec.log("timeseries/battery_v", rerun::Scalars(volts));
}

void RerunSink::log_altitude(float z) {
    impl_->rec.log("timeseries/altitude_m", rerun::Scalars(z));
}

void RerunSink::log_setpoint(const SetpointCommandEvent& e, const VehicleState& v) {
    using K = SetpointCommandEvent::Kind;
    if (e.kind != K::Hover) return;   // only Hover has a meaningful target

    // For now show the commanded z as a scalar on its own time-series
    // and put a dot on the commanded path at the current xy + commanded z.
    impl_->rec.log("timeseries/setpoint_z_target_m",
                   rerun::Scalars(e.z_target_m));
    impl_->rec.log("odom/path_commanded",
        rerun::Points3D({{v.position.x, v.position.y, e.z_target_m}})
            .with_radii({0.008f}));
}

void RerunSink::log_ranges(const RangeSnapshot& r) {
    const auto rays = rays_from_snapshot(r);
    for (const auto& ray : rays) {
        if (!ray.valid) {
            // Clear the previous ray so it doesn't linger on screen.
            impl_->rec.log(sensor_path(ray.sensor),
                rerun::LineStrips3D(
                    std::vector<std::vector<std::array<float, 3>>>{}));
            continue;
        }
        const float ex = ray.origin_x + ray.range_m * ray.dx;
        const float ey = ray.origin_y + ray.range_m * ray.dy;
        const float ez = ray.origin_z + ray.range_m * ray.dz;
        impl_->rec.log(sensor_path(ray.sensor),
            rerun::LineStrips3D(
                std::vector<std::vector<std::array<float, 3>>>{
                    {{ray.origin_x, ray.origin_y, ray.origin_z},
                     {ex, ey, ez}}}));
    }
}

void RerunSink::log_obstacle_point(const ObstaclePoint& p) {
    impl_->rec.log("odom/obstacles/points",
        rerun::Points3D({{p.x, p.y, p.z}})
            .with_radii({0.02f}));
}

void RerunSink::log_mission_state(MissionState s, AbortReason reason,
                                   std::string_view detail) {
    std::string text = std::string{"mission: "} + state_name(s);
    if (s == MissionState::Aborted && reason != AbortReason::None) {
        text += " (";
        switch (reason) {
            case AbortReason::TelemetryStale: text += "telemetry_stale"; break;
            case AbortReason::LowBattery:     text += "low_battery";     break;
            case AbortReason::LinkLost:       text += "link_lost";       break;
            case AbortReason::OperatorAbort:  text += "operator_abort";  break;
            case AbortReason::MissionTimeout: text += "mission_timeout"; break;
            default:                          text += "?";               break;
        }
        if (!detail.empty()) {
            text += ": ";
            text.append(detail);
        }
        text += ")";
    }
    impl_->rec.log("events/mission",
        rerun::TextLog(text)
            .with_level(s == MissionState::Aborted
                            ? rerun::TextLogLevel::Warning
                            : rerun::TextLogLevel::Info));
}

} // namespace cfo
