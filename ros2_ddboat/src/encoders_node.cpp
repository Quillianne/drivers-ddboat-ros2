#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class EncodersNode : public rclcpp::Node {
public:
  EncodersNode() : Node("encoders_node"), serial_() {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyENC0");
    int baud = this->declare_parameter<int>("baud", 115200);
    delay_ms_ = this->declare_parameter<int>("delay", 100);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    auto timeout = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(timeout);
    try {
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(),
                   e.what());
    }
    send_delay_command(delay_ms_);
    send_c_command();
    publisher_ =
        this->create_publisher<std_msgs::msg::Int32MultiArray>("encoders", 10);
    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&EncodersNode::read_loop, this));
    srv_clear_ = this->create_service<std_srvs::srv::Trigger>(
        "clear_counts",
        std::bind(&EncodersNode::clear_cb, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_last_ = this->create_service<std_srvs::srv::Trigger>(
        "request_last",
        std::bind(&EncodersNode::last_cb, this, std::placeholders::_1,
                  std::placeholders::_2));
    param_cb_ = this->add_on_set_parameters_callback(
        std::bind(&EncodersNode::param_cb, this, std::placeholders::_1));
  }

private:
  void read_loop() {
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
    msg.data = {timer,     dir_left,   dir_right,  pos_left,   pos_right,
                volt_left, volt_right, prev_left_, prev_right_};
    publisher_->publish(msg);
    prev_left_ = pos_left;
    prev_right_ = pos_right;
  }

  void send_c_command() {
    if (serial_.isOpen()) {
      serial_.write("C");
    }
  }

  void send_p_command() {
    if (serial_.isOpen()) {
      serial_.write("P");
    }
  }

  void send_delay_command(int d) {
    if (serial_.isOpen()) {
      char buf[16];
      snprintf(buf, sizeof(buf), "D%d;", d);
      serial_.write(std::string(buf));
    }
  }

  void clear_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    send_c_command();
    res->success = true;
    res->message = "C command sent";
  }

  void last_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    send_p_command();
    res->success = true;
    res->message = "P command sent";
  }

  rcl_interfaces::msg::SetParametersResult
  param_cb(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == "delay") {
        delay_ms_ = p.as_int();
        send_delay_command(delay_ms_);
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_last_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  int delay_ms_{};
  int prev_left_{0};
  int prev_right_{0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncodersNode>());
  rclcpp::shutdown();
  return 0;
}
