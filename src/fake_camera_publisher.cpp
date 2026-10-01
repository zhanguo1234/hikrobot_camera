#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class FakeCameraPublisher : public rclcpp::Node
{
public:
  FakeCameraPublisher() : Node("fake_camera_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&FakeCameraPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::Image();
    message.header.stamp = this->now();
    message.height = 480;
    message.width = 640;
    message.encoding = "mono8";
    message.step = 640;
    message.data.resize(480 * 640, 128); // 填充灰色
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing fake image...");
  }
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeCameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
