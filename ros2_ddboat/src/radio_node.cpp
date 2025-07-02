#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class RadioNode : public rclcpp::Node
{
public:
  RadioNode() : Node("radio_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyLORA1");
    int baud = this->declare_parameter<int>("baud", 115200);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
    try {
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), e.what());
    }
    pub_ = this->create_publisher<std_msgs::msg::String>("radio_rx", 10);
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "radio_tx", 10, std::bind(&RadioNode::tx_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&RadioNode::rx_loop, this));
  }

private:
  void tx_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!serial_.isOpen()) return;
    serial_.write(msg->data);
  }

  void rx_loop()
  {
    if (!serial_.isOpen()) return;
    std::string line = serial_.readline();
    if (!line.empty()) {
      std_msgs::msg::String out;
      out.data = line;
      pub_->publish(out);
    }
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadioNode>());
  rclcpp::shutdown();
  return 0;
}
