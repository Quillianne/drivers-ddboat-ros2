#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <serial/serial.h>
#include <string>
#include <chrono>
#include <thread>

class ArduinoNode : public rclcpp::Node
{
public:
  ArduinoNode() : Node("arduino_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyV0");
    int baud = this->declare_parameter<int>("baud", 115200);
    bool calibrate = this->declare_parameter<bool>("calibrate", true);
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
    if (serial_.isOpen() && calibrate) {
      std::string res = calibrate_esc();
      RCLCPP_INFO(get_logger(), "ESC calibration: %s", res.c_str());
    }
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "motors_cmd", 10, std::bind(&ArduinoNode::cmd_callback, this, std::placeholders::_1));

    srv_calib_ = this->create_service<std_srvs::srv::Trigger>(
      "calibrate_esc",
      std::bind(&ArduinoNode::handle_calibrate, this, std::placeholders::_1, std::placeholders::_2));
    srv_cmd_ = this->create_service<std_srvs::srv::Trigger>(
      "get_cmd_motor",
      std::bind(&ArduinoNode::handle_get_cmd, this, std::placeholders::_1, std::placeholders::_2));
    srv_cmdesc_ = this->create_service<std_srvs::srv::Trigger>(
      "get_cmd_motor_esc",
      std::bind(&ArduinoNode::handle_get_cmdesc, this, std::placeholders::_1, std::placeholders::_2));
    srv_rc_ = this->create_service<std_srvs::srv::Trigger>(
      "get_rc_chan",
      std::bind(&ArduinoNode::handle_get_rc, this, std::placeholders::_1, std::placeholders::_2));
    srv_rawrc_ = this->create_service<std_srvs::srv::Trigger>(
      "get_raw_rc_chan",
      std::bind(&ArduinoNode::handle_get_rawrc, this, std::placeholders::_1, std::placeholders::_2));
    srv_status_ = this->create_service<std_srvs::srv::Trigger>(
      "get_status",
      std::bind(&ArduinoNode::handle_get_status, this, std::placeholders::_1, std::placeholders::_2));
    srv_energy_ = this->create_service<std_srvs::srv::Trigger>(
      "get_energy_saver",
      std::bind(&ArduinoNode::handle_get_energy, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~ArduinoNode() override
  {
    if (serial_.isOpen()) {
      serial_.write("M 000 000;");
      serial_.close();
    }

  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!serial_.isOpen()) {
      return;
    }
    int cmdl = bound_cmd(static_cast<int>(msg->linear.x));
    int cmdr = bound_cmd(static_cast<int>(msg->linear.y));
    char dirl = ' ';
    char dirr = ' ';
    if (cmdl < 0) { dirl = '-'; cmdl = -cmdl; }
    if (cmdr < 0) { dirr = '-'; cmdr = -cmdr; }
    char buf[32];
    snprintf(buf, sizeof(buf), "M%c%03d%c%03d;", dirl, cmdl, dirr, cmdr);
    serial_.write(std::string(buf));
  }

  int bound_cmd(int cmd)
  {
    if (cmd > 255) cmd = 255;
    if (cmd < -255) cmd = -255;
    return cmd;
  }

  std::string send_command(const std::string &cmd, double timeout)
  {
    if (!serial_.isOpen()) return "";
    serial_.write(cmd);
    auto start = std::chrono::steady_clock::now();
    while (true) {
      std::string line = serial_.readline();
      if (!line.empty()) return line.substr(0, line.size() - 1);
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count() > timeout) {
        RCLCPP_WARN(get_logger(), "%s timeout %.1f", cmd.c_str(), timeout);
        return "";
      }
    }
  }

  std::string calibrate_esc(double timeout = 1.0)
  {
    if (!serial_.isOpen()) return "";
    serial_.write("I;");
    std::this_thread::sleep_for(std::chrono::seconds(7));
    return send_command("", timeout); // read reply only
  }

  std::string get_arduino_cmd_motor(double timeout = 1.0)
  { return send_command("C;", timeout); }

  std::string get_arduino_cmd_motor_esc(double timeout = 1.0)
  { return send_command("Z;", timeout); }

  std::string get_arduino_rc_chan(double timeout = 1.0)
  { return send_command("R;", timeout); }

  std::string get_arduino_raw_rc_chan(double timeout = 1.0)
  { return send_command("X;", timeout); }

  std::string get_arduino_status(double timeout = 1.0)
  { return send_command("S;", timeout); }

  std::string get_arduino_energy_saver(double timeout = 1.0)
  { return send_command("E;", timeout); }

  void handle_calibrate(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = calibrate_esc();
    res->success = !ans.empty();
    res->message = ans;
  }

  void handle_get_cmd(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = get_arduino_cmd_motor();
    res->success = !ans.empty();
    res->message = ans;
  }

  void handle_get_cmdesc(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = get_arduino_cmd_motor_esc();
    res->success = !ans.empty();
    res->message = ans;
  }

  void handle_get_rc(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = get_arduino_rc_chan();
    res->success = !ans.empty();
    res->message = ans;
  }

  void handle_get_rawrc(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = get_arduino_raw_rc_chan();
    res->success = !ans.empty();
    res->message = ans;
  }

  void handle_get_status(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = get_arduino_status();
    res->success = !ans.empty();
    res->message = ans;
  }

  void handle_get_energy(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    std::string ans = get_arduino_energy_saver();
    res->success = !ans.empty();
    res->message = ans;
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_calib_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cmd_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cmdesc_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_rc_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_rawrc_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_energy_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoNode>());
  rclcpp::shutdown();
  return 0;
}
