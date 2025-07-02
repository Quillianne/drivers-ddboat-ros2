#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <serial/serial.h>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class EncodersNode : public rclcpp::Node
{
public:
  EncodersNode() : Node("encoders_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyENC0");
    int baud = this->declare_parameter<int>("baud", 115200);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
    try {
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), e.what());
    }
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("encoders", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&EncodersNode::read_loop, this));
  }

private:
  void read_loop()
  {
    if (!serial_.isOpen()) {
      return;
    }
    uint8_t buf[17];
    size_t n = serial_.read(buf, 17);
    if (n != 17) {
      return;
    }
    if (buf[0] != 0xff || buf[1] != 0x0d) {
      return;
    }
    int32_t timer = (buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5];
    int dir_left = buf[7];
    int dir_right = buf[6];
    int pos_left = (buf[10] << 8) | buf[11];
    int pos_right = (buf[8] << 8) | buf[9];
    int volt_left = (buf[14] << 8) | buf[15];
    int volt_right = (buf[12] << 8) | buf[13];
    if (buf[16] != 0xaa) {
      return;
    }
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {timer, dir_left, dir_right, pos_left, pos_right, volt_left, volt_right};
    publisher_->publish(msg);
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncodersNode>());
  rclcpp::shutdown();
  return 0;
}
