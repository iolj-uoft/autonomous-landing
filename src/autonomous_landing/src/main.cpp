#include <cstdio>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "autonomous_landing/velocity_setpoint_control.hpp"

int main(int argc, char * argv[])
{
	std::cout << "Starting velocity setpoints control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<autonomous_landing::VelocitySetpointControl>());

	rclcpp::shutdown();
	return 0;
}
