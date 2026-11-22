// Republishes the latest CameraInfo with the header (stamp + frame_id) of every incoming Image,
// so consumers using exact-time sync (image_transport CameraSubscriber, e.g. apriltag_ros)
// get matching pairs even when the simulator stamps the two topics differently.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class CameraInfoSync : public rclcpp::Node
{
public:
	CameraInfoSync() : Node("camera_info_sync")
	{
		const auto qos = rclcpp::SensorDataQoS();
		info_pub_ = create_publisher<CameraInfo>("camera_info_out", qos);

		info_sub_ = create_subscription<CameraInfo>(
			"camera_info_in", qos,
			[this](CameraInfo::ConstSharedPtr msg) { latest_info_ = msg; });

		image_sub_ = create_subscription<Image>(
			"image", qos,
			[this](Image::ConstSharedPtr img) {
				if (!latest_info_) {
					RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
						"Got image but no camera_info yet");
					return;
				}
				CameraInfo out = *latest_info_;
				out.header = img->header;
				info_pub_->publish(out);
			});
	}

private:
	rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
	rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
	rclcpp::Subscription<Image>::SharedPtr image_sub_;
	CameraInfo::ConstSharedPtr latest_info_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraInfoSync>());
	rclcpp::shutdown();
	return 0;
}
