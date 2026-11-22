#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <vector>

#include "autonomous_landing/mission_phase.hpp"
#include "autonomous_landing/velocity_pid.hpp"

namespace autonomous_landing
{

/**
 * @brief Offboard velocity-setpoint node.
 *
 * Waits for a valid local position, streams setpoints, switches to Offboard and arms,
 * flies the waypoint list with a PID velocity controller (advancing after holding each
 * waypoint for ~1 s), then commands a land.
 */
class VelocitySetpointControl : public rclcpp::Node
{
public:
	VelocitySetpointControl();

	void arm();
	void disarm();

private:
	// State machine
	void on_timer();
	void transition_to(Phase next);
	void update_waypoints();

	// Helpers
	bool reached(const Eigen::Vector3f & wp) const;
	void publish_offboard_control_mode();
	void publish_velocity_setpoint(const Eigen::Vector3f & wp);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	uint64_t now_us() const;

	Phase phase_{Phase::WaitForEstimate};

	// Controller
	VelocityPid pid_;
	uint64_t last_us_{0};
	float yaw_sp_{NAN};

	// Vehicle state (from VehicleLocalPosition)
	Eigen::Vector3f pos_{Eigen::Vector3f::Zero()};
	Eigen::Vector3f vel_{Eigen::Vector3f::Zero()};
	float yaw_{0.0f};
	bool position_valid_{false};
	uint8_t arming_state_{px4_msgs::msg::VehicleStatus::ARMING_STATE_INIT};  // from VehicleStatus

	// Mission
	std::vector<Eigen::Vector3f> waypoints_;
	size_t wp_index_{0};
	int hold_ticks_{0};
	int prestream_count_{0};

	static constexpr float kAcceptanceRadius = 0.3f;  // [m]
	static constexpr int kHoldTicks = 10;             // ticks inside radius before advancing (~1 s)
	static constexpr int kPrestreamTicks = 10;        // setpoints sent before switching to Offboard

	// ROS
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
};

}  // namespace autonomous_landing
