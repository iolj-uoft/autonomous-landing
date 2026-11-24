/**
 * Keyboard teleop for PX4 over offboard mode.
 *
 *   W / S        forward / backward   (body frame, "pitch")
 *   A / D        left / right         (body frame, "roll")
 *   Q / E        yaw left / right
 *   Space        climb   ("thrust up")
 *   Left Ctrl    descend ("thrust down")
 *   T            switch to Offboard + arm
 *   L            land
 *
 * With no keys held the drone hovers: each axis group (horizontal, vertical, yaw) latches
 * its current position/heading once the vehicle has slowed down and holds it with a
 * position setpoint. Held keys switch that axis group to velocity / yaw-rate control.
 *
 * Keys are read with XQueryKeymap (X11) because a terminal cannot report key releases or a
 * bare Ctrl press. This reads the GLOBAL keyboard state, so keys are picked up no matter
 * which window has focus.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

// X11 after ROS: Xlib #defines names (None, Status, Bool, ...) that collide with rclcpp
#include <X11/Xlib.h>
#include <X11/keysym.h>
#undef None
#undef Status
#undef Bool

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace autonomous_landing
{

/// Snapshot of the global keyboard state via X11.
class KeyboardState
{
public:
	KeyboardState()
	{
		display_ = XOpenDisplay(nullptr);
		if (!display_) {
			throw std::runtime_error("Cannot open X display (is $DISPLAY set?)");
		}
	}
	~KeyboardState() { XCloseDisplay(display_); }
	KeyboardState(const KeyboardState &) = delete;
	KeyboardState & operator=(const KeyboardState &) = delete;

	void poll() { XQueryKeymap(display_, keys_.data()); }

	bool down(KeySym sym) const
	{
		const KeyCode kc = XKeysymToKeycode(display_, sym);
		return kc != 0 && (keys_[kc / 8] & (1 << (kc % 8)));
	}

private:
	Display * display_{nullptr};
	std::array<char, 32> keys_{};
};

class KeyboardTeleop : public rclcpp::Node
{
public:
	KeyboardTeleop() : Node("keyboard_teleop")
	{
		max_xy_speed_ = declare_parameter("max_xy_speed", 2.0);          // [m/s]
		max_z_speed_ = declare_parameter("max_z_speed", 1.0);            // [m/s]
		max_yaw_rate_ = declare_parameter("max_yaw_rate_deg", 45.0) * M_PI / 180.0;  // [rad/s]

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		local_position_sub_ = create_subscription<VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position", qos,
			[this](const VehicleLocalPosition::UniquePtr msg) {
				x_ = msg->x; y_ = msg->y; z_ = msg->z;
				vx_ = msg->vx; vy_ = msg->vy; vz_ = msg->vz;
				yaw_ = msg->heading;
				position_valid_ = msg->xy_valid && msg->z_valid;
			});

		vehicle_status_sub_ = create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status", qos,
			[this](const VehicleStatus::UniquePtr msg) {
				nav_state_ = msg->nav_state;
			});

		offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		timer_ = create_wall_timer(50ms, [this]() { on_timer(); });

		RCLCPP_INFO(get_logger(),
			"\nKeyboard teleop\n"
			"  W/S  forward/back     A/D  left/right     Q/E  yaw left/right\n"
			"  Space  climb          Left Ctrl  descend\n"
			"  T  offboard + arm     L  land\n"
			"No keys held -> hover. NOTE: keys are read globally, regardless of window focus.");
	}

private:
	void on_timer()
	{
		kb_.poll();
		handle_commands();

		// Don't stream anything until we know where we are: hover latches need a valid position
		if (!position_valid_) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for a valid local position...");
			return;
		}

		// Outside Offboard the latches are stale (e.g. after a land/disarm); re-latch on entry
		if (nav_state_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
			hold_xy_.reset();
			hold_z_.reset();
			hold_yaw_.reset();
		}

		publish_offboard_control_mode();
		publish_setpoint();
	}

	/// One-shot actions on key press (rising edge).
	void handle_commands()
	{
		const bool t = kb_.down(XK_t);
		const bool l = kb_.down(XK_l);

		if (t && !prev_t_) {
			if (!position_valid_) {
				RCLCPP_WARN(get_logger(), "No valid position yet, not arming");
			} else {
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // Offboard
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
				RCLCPP_INFO(get_logger(), "Offboard + arm sent");
			}
		}
		if (l && !prev_l_) {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
			RCLCPP_INFO(get_logger(), "Land sent");
		}
		prev_t_ = t;
		prev_l_ = l;
	}

	static float axis(bool pos, bool neg) { return static_cast<float>(pos) - static_cast<float>(neg); }

	void publish_setpoint()
	{
		constexpr float nan = std::numeric_limits<float>::quiet_NaN();

		// Body-frame commands, normalised to [-1, 1]
		const float fwd = axis(kb_.down(XK_w), kb_.down(XK_s));
		const float right = axis(kb_.down(XK_d), kb_.down(XK_a));
		const float up = axis(kb_.down(XK_space), kb_.down(XK_Control_L));
		const float yaw_cw = axis(kb_.down(XK_e), kb_.down(XK_q));  // NED: positive yaw = clockwise

		TrajectorySetpoint msg{};
		msg.position = {nan, nan, nan};
		msg.velocity = {nan, nan, nan};
		msg.yaw = nan;
		msg.yawspeed = nan;

		// Horizontal: body-frame velocity rotated into NED, or hold position
		if (fwd != 0.0f || right != 0.0f) {
			hold_xy_.reset();
			const float c = std::cos(yaw_), s = std::sin(yaw_);
			const float scale = static_cast<float>(max_xy_speed_) / std::hypot(fwd, right);  // diagonals not faster
			msg.velocity[0] = (c * fwd - s * right) * scale;
			msg.velocity[1] = (s * fwd + c * right) * scale;
		} else if (!hold_xy_ && std::hypot(vx_, vy_) > kSettleSpeedXY) {
			// Brake first, latch once slow: latching at speed would pull the drone back
			msg.velocity[0] = 0.0f;
			msg.velocity[1] = 0.0f;
		} else {
			if (!hold_xy_) { hold_xy_ = std::array<float, 2>{x_, y_}; }
			msg.position[0] = (*hold_xy_)[0];
			msg.position[1] = (*hold_xy_)[1];
		}

		// Vertical (NED: negative z is up)
		if (up != 0.0f) {
			hold_z_.reset();
			msg.velocity[2] = -up * static_cast<float>(max_z_speed_);
		} else if (!hold_z_ && std::abs(vz_) > kSettleSpeedZ) {
			msg.velocity[2] = 0.0f;
		} else {
			if (!hold_z_) { hold_z_ = z_; }
			msg.position[2] = *hold_z_;
		}

		// Yaw
		if (yaw_cw != 0.0f) {
			hold_yaw_.reset();
			msg.yawspeed = yaw_cw * static_cast<float>(max_yaw_rate_);
		} else {
			if (!hold_yaw_) { hold_yaw_ = yaw_; }
			msg.yaw = *hold_yaw_;
		}

		msg.timestamp = now_us();
		trajectory_setpoint_pub_->publish(msg);
	}

	void publish_offboard_control_mode()
	{
		// Position + velocity: PX4 uses whichever setpoint field is finite per axis
		OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = true;
		msg.timestamp = now_us();
		offboard_control_mode_pub_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
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

	uint64_t now_us() const { return now().nanoseconds() / 1000; }

	static constexpr float kSettleSpeedXY = 0.3f;  // [m/s] below this, latch horizontal hold
	static constexpr float kSettleSpeedZ = 0.2f;   // [m/s] below this, latch altitude hold

	KeyboardState kb_;
	bool prev_t_{false};
	bool prev_l_{false};

	double max_xy_speed_;
	double max_z_speed_;
	double max_yaw_rate_;

	// Vehicle state
	float x_{0}, y_{0}, z_{0}, vx_{0}, vy_{0}, vz_{0}, yaw_{0};
	bool position_valid_{false};
	uint8_t nav_state_{0};

	// Hover latches (empty = not holding)
	std::optional<std::array<float, 2>> hold_xy_;
	std::optional<float> hold_z_;
	std::optional<float> hold_yaw_;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
};

}  // namespace autonomous_landing

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<autonomous_landing::KeyboardTeleop>());
	rclcpp::shutdown();
	return 0;
}
