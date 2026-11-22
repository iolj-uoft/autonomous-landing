#pragma once

#if __has_include(<Eigen/Dense>)
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

namespace autonomous_landing {

struct PidGains {
    float kp{2.8f};
    float ki{0.1f};
    float kd{1.5f};
    float i_max{0.5f};
    float v_max{4.0f};
};

struct PidOutput
{
    Eigen::Vector3f velocity;
    bool saturated;
};

class VelocityPid 
{
public: 
    explicit VelocityPid(const PidGains& gains = {}) : gains_(gains) {} // initialize the gains_ member with PidGains struct's value

    PidOutput update(const Eigen::Vector3f & err, const Eigen::Vector3f & vel, float dt);
    void reset_integral_error() { integral_.setZero(); }
    void set_gains(const PidGains & g) { gains_ = g; }
private:
    PidGains gains_;
    Eigen::Vector3f integral_{Eigen::Vector3f::Zero()};
};    
    
} // namespace autonomous_landing