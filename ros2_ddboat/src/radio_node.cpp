#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <string>
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

class RadioNode : public rclcpp::Node
{
public:
  RadioNode() : Node("radio_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyLORA1");
    int baud = this->declare_parameter<int>("baud", 115200);
    id_src_ = this->declare_parameter<int>("id_src", 1);
    id_dst_ = this->declare_parameter<int>("id_dst", 2);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    auto timeout = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(timeout);
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
    std::ostringstream frame;
    frame << id_src_ << ':' << id_dst_ << ':' << msg->data.size() << ':' << msg->data << '\n';
    serial_.write(frame.str());
  }

  static std::vector<std::string> split(const std::string &str, char sep)
  {
    std::vector<std::string> out;
    std::string tmp;
    for (char c : str) {
      if (c == sep) {
        out.push_back(tmp);
        tmp.clear();
      } else {
        tmp += c;
      }
    }
    out.push_back(tmp);
    return out;
  }

  void rx_loop()
  {
    if (!serial_.isOpen()) return;
    std::string line = serial_.readline();
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
      line.pop_back();
    if (line.empty()) return;
    auto parts = split(line, ':');
    if (parts.size() < 4) return;
    try {
      int src = std::stoi(parts[0]);
      int dst = std::stoi(parts[1]);
      int len = std::stoi(parts[2]);
      std::string payload = parts[3];
      if ((int)payload.size() > len) {
        payload = payload.substr(0, len);
      }
      if (dst == id_src_) {
        std_msgs::msg::String out;
        out.data = payload;
        pub_->publish(out);
      }
    } catch (const std::exception &e) {
      // ignore malformed line
    }
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int id_src_{1};
  int id_dst_{2};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadioNode>());
  rclcpp::shutdown();
  return 0;
}
