// Smallest possible compile/link check for the Crazyflie link adapter.
//
// Goal: prove that cfo_link compiles, links against crazyflie-link-cpp, and
// exposes only its public (vendor-free) interface. No decoding, no hardware
// required — scan() returns an empty vector when no Crazyradio is attached.

#include "crazyflie/link/interfaces.hpp"
#include "crazyflie/link/types.hpp"

#include <cstdio>

int main() {
    const auto uris = cfo::scan_crazyflie_links();
    std::printf("cfo_link ok; scan returned %zu uri(s)\n", uris.size());

    // Exercise the type — confirms RawPacket and the factory are visible.
    cfo::RawPacket pkt{};
    pkt.port = 0;
    pkt.channel = 0;
    pkt.size = 0;

    auto link = cfo::open_crazyflie_link("radio://0/80/2M/E7E7E7E7E7");
    if (!link) {
        std::printf("open: not connected (expected without hardware)\n");
        return 0;
    }
    (void)(*link)->send(pkt);
    return 0;
}
