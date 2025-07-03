#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std::chrono_literals;

class TempNode : public rclcpp::Node
{
public:
  TempNode() : Node("temperature_node")
  {
    bus_ = this->declare_parameter<std::string>("bus", "/dev/i2c-1");
    open_bus();
    pub_left_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature_left", 10);
    pub_right_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature_right", 10);
    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("temperature_diag", 10);
    srv_standby_ = this->create_service<std_srvs::srv::SetBool>(
      "set_standby",
      std::bind(&TempNode::handle_set_standby, this, std::placeholders::_1, std::placeholders::_2));
    srv_get_cfg_ = this->create_service<std_srvs::srv::Trigger>(
      "get_config",
      std::bind(&TempNode::handle_get_config, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(1s, std::bind(&TempNode::read_loop, this));
  }

  ~TempNode() override
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  void open_bus()
  {
    fd_ = open(bus_.c_str(), O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", bus_.c_str());
    }
  }

  bool read_reg(uint8_t addr, uint8_t reg, uint8_t &val)
  {
    if (ioctl(fd_, I2C_SLAVE, addr) < 0) return false;
    if (write(fd_, &reg, 1) != 1) return false;
    if (read(fd_, &val, 1) != 1) return false;
    return true;
  }

  bool write_reg(uint8_t addr, uint8_t reg, uint8_t val)
  {
    if (ioctl(fd_, I2C_SLAVE, addr) < 0) return false;
    uint8_t buf[2] = {reg, val};
    if (write(fd_, buf, 2) != 2) return false;
    return true;
  }

  bool set_standby(uint8_t addr, bool standby)
  {
    uint8_t cfg;
    if (!read_reg(addr, 0x01, cfg)) return false;
    if (standby)
      cfg |= 0x80;
    else
      cfg &= ~0x80;
    return write_reg(addr, 0x01, cfg);
  }

  bool get_config(uint8_t addr, uint8_t &cfg)
  {
    return read_reg(addr, 0x01, cfg);
  }

  void read_loop()
  {
    sensor_msgs::msg::Temperature tleft;
    sensor_msgs::msg::Temperature tright;
    uint8_t val;
    if (read_reg(0x4d, 0x00, val)) {
      tleft.temperature = static_cast<int8_t>(val);
    } else {
      err_left_++;
    }
    if (read_reg(0x48, 0x00, val)) {
      tright.temperature = static_cast<int8_t>(val);
    } else {
      err_right_++;
    }
    pub_left_->publish(tleft);
    pub_right_->publish(tright);

    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "temperature_node";
    diag.hardware_id = "tc74";
    diag.level = (err_left_ || err_right_) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                            : diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag.message = "I2C read status";
    diagnostic_msgs::msg::KeyValue kv_l;
    kv_l.key = "left_errors";
    kv_l.value = std::to_string(err_left_);
    diag.values.push_back(kv_l);
    diagnostic_msgs::msg::KeyValue kv_r;
    kv_r.key = "right_errors";
    kv_r.value = std::to_string(err_right_);
    diag.values.push_back(kv_r);
    pub_diag_->publish(diag);
  }

  std::string bus_;
  int fd_{-1};
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_right_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_standby_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_get_cfg_;
  int err_left_{0};
  int err_right_{0};

  void handle_set_standby(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    bool ok_left = set_standby(0x4d, req->data);
    bool ok_right = set_standby(0x48, req->data);
    res->success = ok_left && ok_right;
    res->message = res->success ? "ok" : "failed";
  }

  void handle_get_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    uint8_t cfg_l = 0, cfg_r = 0;
    bool ok_left = get_config(0x4d, cfg_l);
    bool ok_right = get_config(0x48, cfg_r);
    res->success = ok_left && ok_right;
    char buf[64];
    snprintf(buf, sizeof(buf), "left=0x%02X right=0x%02X", cfg_l, cfg_r);
    res->message = buf;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TempNode>());
  rclcpp::shutdown();
  return 0;
}
