#pragma once

#include "types.hpp"

#include <mutex>

namespace cfo {

// The single authoritative state holder.
//
// Thread-safe via a plain mutex. Readers obtain a value-copy snapshot;
// writers feed telemetry through the pure `apply` overloads under the lock.
// No callbacks, no observers — consumers pull snapshots when they need them.
class StateStore {
public:
    StateStore() = default;
    explicit StateStore(VehicleState initial) : state_{initial} {}

    StateStore(const StateStore&) = delete;
    StateStore& operator=(const StateStore&) = delete;

    [[nodiscard]] VehicleState snapshot() const {
        std::lock_guard lock{m_};
        return state_;
    }

    void apply(const Telemetry& msg) {
        std::lock_guard lock{m_};
        state_ = cfo::apply(state_, msg);
    }

private:
    mutable std::mutex m_;
    VehicleState state_{};
};

} // namespace cfo
