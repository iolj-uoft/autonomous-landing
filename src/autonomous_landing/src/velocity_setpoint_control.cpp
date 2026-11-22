#include "autonomous_landing/velocity_setpoint_control.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace autonomous_landing
{

VelocitySetpointControl::VelocitySetpointControl() : Node("velocity_setpoint_control")
{
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	local_position_sub_ = create_subscription<VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", qos,
		[this](const VehicleLocalPosition::UniquePtr msg) {
			pos_ = {msg->x, msg->y, msg->z};
			vel_ = {msg->vx, msg->vy, msg->vz};
			yaw_ = msg->heading;
			position_valid_ = msg->xy_valid && msg->z_valid;
		});

	// PX4 v1.14 doesn't bridge vehicle_land_detected; it auto-disarms after touchdown
	// (COM_DISARM_LAND), so a disarm while in Landing means we're on the ground.
	vehicle_status_sub_ = create_subscription<VehicleStatus>(
		"/fmu/out/vehicle_status", qos,
		[this](const VehicleStatus::UniquePtr msg) {
			arming_state_ = msg->arming_state;
		});

	waypoints_ = {
		{0.0f, 0.0f, -5.0f},
		{20.0f, 0.0f, -5.0f},
		{20.0f, 20.0f, -5.0f},
		{0.0f, 20.0f, -5.0f},
		{0.0f, 0.0f, -5.0f},
	};

	offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

	timer_ = create_wall_timer(100ms, [this]() { on_timer(); });
}

void VelocitySetpointControl::on_timer()
{
	// PX4 needs offboard_control_mode + trajectory_setpoint streamed continuously while in Offboard
	if (phase_ != Phase::Landing && phase_ != Phase::Done && phase_ != Phase::Abort) {
		publish_offboard_control_mode();
		publish_velocity_setpoint(waypoints_[wp_index_]);
	}

	switch (phase_) {
		case Phase::WaitForEstimate:
			if (position_valid_) {
				transition_to(Phase::Prestream);
			}
			break;

		case Phase::Prestream:
			if (++prestream_count_ >= kPrestreamTicks) {
				transition_to(Phase::Arming);
			}
			break;

		case Phase::Arming:
			transition_to(Phase::Waypoints);
			break;

		case Phase::Waypoints:
			update_waypoints();
			break;

		case Phase::Landing:
			if (arming_state_ == VehicleStatus::ARMING_STATE_STANDBY) {
				transition_to(Phase::Done);
			}
			break;

		case Phase::Done:
		case Phase::Abort:
			break;
	}
}

void VelocitySetpointControl::transition_to(Phase next)
{
	RCLCPP_INFO(get_logger(), "Phase: %s -> %s", to_str(phase_), to_str(next));
	phase_ = next;

	// One-shot actions on entering a phase
	switch (next) {
		case Phase::Arming:
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // Offboard
			arm();
			break;

		case Phase::Waypoints:
			// Discard the integral built up while sitting on the ground
			pid_.reset_integral_error();
			hold_ticks_ = 0;
			break;

		case Phase::Landing:
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
			break;

		case Phase::Done:
			RCLCPP_INFO(get_logger(), "Mission complete!");
			break;
			
		default:
			break;
	}
}

void VelocitySetpointControl::update_waypoints()
{
	if (!position_valid_ || !reached(waypoints_[wp_index_])) {
		hold_ticks_ = 0;
		return;
	}

	if (++hold_ticks_ < kHoldTicks) {
		return;
	}
	hold_ticks_ = 0;

	if (wp_index_ + 1 < waypoints_.size()) {
		wp_index_++;
		pid_.reset_integral_error();  // prevent windup carrying over to the next leg
		RCLCPP_INFO(get_logger(), "Waypoint reached -> going to %zu", wp_index_);
	} else {
		RCLCPP_INFO(get_logger(), "Mission complete, landing!");
		transition_to(Phase::Landing);
	}
}

bool VelocitySetpointControl::reached(const Eigen::Vector3f & wp) const
{
	return (wp - pos_).norm() < kAcceptanceRadius;
}

void VelocitySetpointControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
	RCLCPP_INFO(get_logger(), "Arm command sent");
}

void VelocitySetpointControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
	RCLCPP_INFO(get_logger(), "Disarm command sent");
}

uint64_t VelocitySetpointControl::now_us() const
{
	return now().nanoseconds() / 1000;
}

void VelocitySetpointControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = now_us();
	offboard_control_mode_pub_->publish(msg);
}

void VelocitySetpointControl::publish_velocity_setpoint(const Eigen::Vector3f & wp)
{
	const uint64_t t_us = now_us();
	float dt = (last_us_ == 0) ? 0.1f : static_cast<float>(t_us - last_us_) * 1e-6f;
	dt = std::clamp(dt, 0.001f, 0.5f);
	last_us_ = t_us;

	const Eigen::Vector3f err = wp - pos_;
	const PidOutput out = pid_.update(err, vel_, dt);

	if (out.saturated) {
		RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "v_sp saturated, scaled to |v|=%.2f",
			out.velocity.norm());
	}

	// Face the direction of travel; hold the last heading when close to the waypoint
	const float horiz_dist = std::hypot(err.x(), err.y());
	if (horiz_dist > 0.5f || std::isnan(yaw_sp_)) {
		yaw_sp_ = std::atan2(err.y(), err.x());
	}

	const float nan = std::numeric_limits<float>::quiet_NaN();
	TrajectorySetpoint msg{};
	msg.position = {nan, nan, nan};
	msg.velocity = {out.velocity.x(), out.velocity.y(), out.velocity.z()};
	msg.yaw = std::isnan(yaw_sp_) ? yaw_ : yaw_sp_;
	msg.timestamp = t_us;
	trajectory_setpoint_pub_->publish(msg);
}

void VelocitySetpointControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = now_us();
	vehicle_command_pub_->publish(msg);
}

}  // namespace autonomous_landing
