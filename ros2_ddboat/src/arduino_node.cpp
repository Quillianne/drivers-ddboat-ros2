#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <string>

class ArduinoNode : public rclcpp::Node
{
public:
  ArduinoNode() : Node("arduino_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyV0");
    int baud = this->declare_parameter<int>("baud", 115200);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    auto timeout = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(timeout);
    try {
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), e.what());
      if (port_ == "/dev/ttyV0") {
        RCLCPP_WARN(get_logger(), "Retrying with /dev/ttyACM0");
        try {
          serial_.setPort("/dev/ttyACM0");
          serial_.open();
        } catch (const std::exception &e2) {
          RCLCPP_ERROR(get_logger(), "Failed to open /dev/ttyACM0: %s", e2.what());
        }
      }
    }
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "motors_cmd", 10, std::bind(&ArduinoNode::cmd_callback, this, std::placeholders::_1));
    rclcpp::on_shutdown([this]() { this->send_stop(); });
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!serial_.isOpen()) {
      return;
    }
    int cmdl = static_cast<int>(msg->linear.x);
    int cmdr = static_cast<int>(msg->linear.y);
    char dirl = ' ';
    char dirr = ' ';
    if (cmdl < 0) { dirl = '-'; cmdl = -cmdl; }
    if (cmdr < 0) { dirr = '-'; cmdr = -cmdr; }
    char buf[32];
    snprintf(buf, sizeof(buf), "M%c%03d%c%03d;", dirl, cmdl, dirr, cmdr);
    serial_.write(std::string(buf));
  }

  void send_stop()
  {
    if (!serial_.isOpen()) {
      return;
    }
    serial_.write(std::string("M 000 000;"));
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoNode>());
  rclcpp::shutdown();
  return 0;
}
