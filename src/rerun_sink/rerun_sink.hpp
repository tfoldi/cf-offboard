#pragma once

#include "logging/types.hpp"
#include "mission/types.hpp"
#include "perception/types.hpp"
#include "safety/types.hpp"
#include "state/types.hpp"

#include <expected>
#include <memory>
#include <string>
#include <string_view>

namespace cfo {

// Live visualization sink fed from the same semantic event/state model as
// the MCAP logger. Best-effort: if the rerun viewer is not running, log
// calls degrade quietly. Flight behaviour must remain unaffected by the
// sink's state — see RERUN.md.
//
// Instances are thread-safe (Rerun's RecordingStream is). The sink is
// addressed by the rest of the app through this façade so the rerun
// header dependency stays confined to rerun_sink.cpp.
class RerunSink {
public:
    enum class Mode {
        Spawn,    // auto-launch the viewer if it isn't running
        Connect,  // connect to a viewer assumed to be already running
    };

    static std::expected<std::unique_ptr<RerunSink>, std::string>
    create(std::string_view app_id, Mode mode = Mode::Spawn);

    ~RerunSink();
    RerunSink(const RerunSink&) = delete;
    RerunSink& operator=(const RerunSink&) = delete;

    // Vehicle pose (transform of base_link in odom) + a single point on
    // the actual path trail.
    void log_vehicle(const VehicleState& v);

    // Battery + altitude scalars on dedicated time-series entities.
    void log_battery(float volts);
    void log_altitude(float z);

    // Outbound setpoint markers.
    void log_setpoint(const SetpointCommandEvent& e, const VehicleState& v);

    // Five Multiranger rays anchored at base_link. Invalid rays are
    // skipped so the viewer doesn't show meaningless arrows.
    void log_ranges(const RangeSnapshot& r);

    // Append-only stream of projected obstacle points in odom.
    void log_obstacle_point(const ObstaclePoint& p);

    // Mission state transition as a text annotation entity. Includes the
    // abort reason when entering Aborted.
    void log_mission_state(MissionState s, AbortReason reason,
                           std::string_view detail);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    explicit RerunSink(std::unique_ptr<Impl> impl);
};

} // namespace cfo
