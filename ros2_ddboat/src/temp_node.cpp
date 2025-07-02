#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
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

  void read_loop()
  {
    sensor_msgs::msg::Temperature tleft;
    sensor_msgs::msg::Temperature tright;
    uint8_t val;
    if (read_reg(0x4d, 0x00, val)) tleft.temperature = static_cast<int8_t>(val);
    if (read_reg(0x48, 0x00, val)) tright.temperature = static_cast<int8_t>(val);
    pub_left_->publish(tleft);
    pub_right_->publish(tright);
  }

  std::string bus_;
  int fd_{-1};
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_right_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TempNode>());
  rclcpp::shutdown();
  return 0;
}
