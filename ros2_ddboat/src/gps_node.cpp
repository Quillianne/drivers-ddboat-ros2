#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <serial/serial.h>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class GPSNode : public rclcpp::Node
{
public:
  GPSNode() : Node("gps_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyGPS0");
    int baud = this->declare_parameter<int>("baud", 9600);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
    try {
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open port %s: %s", port_.c_str(), e.what());
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    timer_ = this->create_wall_timer(200ms, std::bind(&GPSNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!serial_.isOpen()) {
      return;
    }
    std::string line = serial_.readline();
    if (line.rfind("$GPGLL", 0) == 0) {
      // TODO: parse line properly
      auto msg = sensor_msgs::msg::NavSatFix();
      publisher_->publish(msg);
    }
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNode>());
  rclcpp::shutdown();
  return 0;
}
