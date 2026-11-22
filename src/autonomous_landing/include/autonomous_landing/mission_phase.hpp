#pragma once

#include <cstdint>

namespace autonomous_landing
{

enum class Phase : uint8_t
{
    WaitForEstimate,  // no valid local position yet
    Prestream,        // streaming setpoints so PX4 accepts the Offboard switch
    Arming,           // send Offboard mode + arm
    Waypoints,        // tracking the waypoint list
    Landing,          // NAV_LAND sent
    Done,
    Abort
};

inline const char * to_str(Phase p)
{
    // No default case: -Wall warns if a new Phase is not handled here
    switch (p) {
        case Phase::WaitForEstimate: return "WaitForEstimate";
        case Phase::Prestream:       return "Prestream";
        case Phase::Arming:          return "Arming";
        case Phase::Waypoints:       return "Waypoints";
        case Phase::Landing:         return "Landing";
        case Phase::Done:            return "Done";
        case Phase::Abort:           return "Abort";
    }
    return "?";
}

}  // namespace autonomous_landing
