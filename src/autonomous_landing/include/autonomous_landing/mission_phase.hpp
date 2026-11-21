#pragma once

namespace autonomous_landing
{
    enum class Phase { WaitForEstimate, Prestream, Arming, Waypoints, Landing, Done, Abort };

    inline const char* to_str(Phase p) { switch (p) {/* ... */} return "?"; }
    
} // namespace autonomous_landing
