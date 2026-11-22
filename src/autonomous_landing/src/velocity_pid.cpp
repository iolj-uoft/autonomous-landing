#include "autonomous_landing/velocity_pid.hpp"

namespace autonomous_landing
{

PidOutput VelocityPid::update(const Eigen::Vector3f & err, const Eigen::Vector3f & vel, float dt)
{
    // I term is clamped per axis; D term acts on measured velocity (no derivative kick on waypoint change)
    const Eigen::Vector3f i_term =
        (gains_.ki * integral_).cwiseMax(-gains_.i_max).cwiseMin(gains_.i_max);
    const Eigen::Vector3f v_raw = gains_.kp * err + i_term - gains_.kd * vel;

    PidOutput out{v_raw, false};

    // Scale the whole vector down if it exceeds v_max (keeps direction)
    const float n = v_raw.norm();
    if (n > gains_.v_max) {
        out.velocity *= gains_.v_max / n;
        out.saturated = true;
    }

    // Conditional integration: don't integrate an axis that is pushing further into saturation
    for (int i = 0; i < 3; ++i) {
        const bool pushes_further = out.saturated && (err[i] * v_raw[i] > 0.0f);
        if (!pushes_further) {
            integral_[i] += err[i] * dt;
        }
    }

    // Bound the accumulator itself so it can't grow without limit while unsaturated
    if (gains_.ki > 0.0f) {
        const float lim = gains_.i_max / gains_.ki;
        integral_ = integral_.cwiseMax(-lim).cwiseMin(lim);
    }

    return out;
}

}  // namespace autonomous_landing
