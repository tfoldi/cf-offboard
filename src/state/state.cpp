#include "types.hpp"

#include <utility>
#include <variant>

namespace cfo {

namespace {

VehicleState bump(VehicleState s, VehicleState::time_point t) {
    s.last_update = t;
    s.update_count += 1;
    return s;
}

} // namespace

VehicleState apply(VehicleState s, const PoseTelemetry& m) {
    s.position = m.position;
    s.attitude = m.attitude;
    return bump(std::move(s), m.t);
}

VehicleState apply(VehicleState s, const VelocityTelemetry& m) {
    s.velocity = m.velocity;
    return bump(std::move(s), m.t);
}

VehicleState apply(VehicleState s, const ImuTelemetry& m) {
    s.angular_rate = m.angular_rate;
    return bump(std::move(s), m.t);
}

VehicleState apply(VehicleState s, const FlowTelemetry& m) {
    s.height_above_ground = m.height_above_ground;
    return bump(std::move(s), m.t);
}

VehicleState apply(VehicleState s, const BatteryTelemetry& m) {
    s.battery_voltage = m.voltage;
    return bump(std::move(s), m.t);
}

VehicleState apply(VehicleState s, const LinkTelemetry& m) {
    s.link = m.status;
    return bump(std::move(s), m.t);
}

VehicleState apply(VehicleState s, const Telemetry& m) {
    return std::visit(
        [&](const auto& msg) { return apply(std::move(s), msg); }, m);
}

} // namespace cfo
