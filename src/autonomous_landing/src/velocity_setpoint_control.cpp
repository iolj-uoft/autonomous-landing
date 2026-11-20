/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <array>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class VelocitySetpointControl : public rclcpp::Node
{
public:
	/**
	 * @brief Construct the offboard_control node.
	 *
	 * Subscribes to the vehicle's local position, loads the square waypoint mission,
	 * creates the offboard/trajectory/command publishers, and starts a 10 Hz timer that:
	 *   - streams setpoints, switches to Offboard mode and arms after 10 setpoints,
	 *   - advances to the next waypoint once it's held for ~1 s (10 ticks),
	 *   - commands a land after the final waypoint.
	 */
	VelocitySetpointControl() : Node("velocity_setpoint_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, [this](const VehicleLocalPosition::UniquePtr msg) {
			current_pos_ = {msg->x, msg->y, msg->z};
			position_valid_ = msg->xy_valid && msg->z_valid;
		});

		if (!position_valid_) {
			current_pos_ = {0.0f, 0.0f, 0.0f};
		}

		waypoints_ = {
			{0.0f, 0.0f, -5.0f},
			{5.0f, 0.0f, -5.0f},
			{5.0f, 5.0f, -5.0f},
			{0.0f, 5.0f, -5.0f},
			{0.0f, 0.0f, -5.0f},
		};

		offboard_setpoint_counter_ = 0;

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// timer_callback is a lambda function stored in a local var in constructor 
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint(waypoints_[wp_index_]);

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
				return;
			}

			if (position_valid_ && reached(waypoints_[wp_index_])) {
				if (++hold_ticks_ >= 10) {
					hold_ticks_ = 0;
					if (wp_index_ + 1 < waypoints_.size()) {
						wp_index_++;
						RCLCPP_INFO(get_logger(), "Waypoint reached -> going to %zu", wp_index_);
					} else if (!mission_done_) {
						mission_done_ = true;
						RCLCPP_INFO(get_logger(), "Mission complete, landing!");
						publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
					}
				}
			} else {
				hold_ticks_ = 0;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();
	bool reached(const std::array<float, 3> &wp) const;


private:
	static constexpr float kAcceptanceRadius = 0.3f; 
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscriber_;
	std::vector<std::array<float, 3>> waypoints_;
	std::array<float, 3> current_pos_{0, 0, 0};
	size_t wp_index_{0};
	int hold_ticks_{0};
	bool position_valid_{false};
	bool mission_done_{false};

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(const std::array<float, 3> &p);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

// @brief Detect if the drone is in the radius of a waypoint
bool VelocitySetpointControl::reached(const std::array<float, 3> &wp) const {
	float dx = wp[0] - current_pos_[0];
	float dy = wp[1] - current_pos_[1];
	float dz = wp[2] - current_pos_[2];
	return std::sqrt(dx*dx + dy*dy + dz*dz) < kAcceptanceRadius;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void VelocitySetpointControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void VelocitySetpointControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void VelocitySetpointControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void VelocitySetpointControl::publish_trajectory_setpoint(const std::array<float, 3> &p)
{	
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float kKp = 1.5;
	const float kV_max = 4.0;
	float pos_error_x = p[0] - current_pos_[0];
	float pos_error_y = p[1] - current_pos_[1];
	float pos_error_z = p[2] - current_pos_[2];
	
	std::array<float, 3> v_sp{std::clamp(kKp * (pos_error_x), -kV_max, kV_max),  std::clamp(kKp * (pos_error_y), -kV_max, kV_max), std::clamp(kKp * (pos_error_z), -kV_max, kV_max)};
	float n = std::sqrt(v_sp[0]*v_sp[0] + v_sp[1]*v_sp[1] + v_sp[2]*v_sp[2]);
	if (n > kV_max) for (auto &elem : v_sp) elem *= kV_max / n;
	
	TrajectorySetpoint msg{};
	msg.position = {nan, nan, nan};
	msg.velocity = {v_sp[0], v_sp[1], v_sp[2]};
	msg.yaw = 3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
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
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting velocity setpoints control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocitySetpointControl>());

	rclcpp::shutdown();
	return 0;
}
