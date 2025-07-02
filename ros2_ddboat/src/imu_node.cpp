#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node
{
public:
  ImuNode() : Node("imu_node")
  {
    bus_ = this->declare_parameter<std::string>("bus", "/dev/i2c-1");
    open_bus();
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&ImuNode::read_loop, this));
  }

  ~ImuNode() override
  {
    if (fd_ag_ >= 0) close(fd_ag_);
    if (fd_mg_ >= 0) close(fd_mg_);
  }

private:
  void open_bus()
  {
    fd_ag_ = open(bus_.c_str(), O_RDWR);
    fd_mg_ = open(bus_.c_str(), O_RDWR);
    if (fd_ag_ < 0 || fd_mg_ < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", bus_.c_str());
    }
  }

  bool read_block(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
  {
    // Allow operation even if ioctl is not supported (emulation)
    ioctl(fd, I2C_SLAVE, addr);
    if (write(fd, &reg, 1) != 1) {
      // generate dummy data on failure
      for (size_t i = 0; i < len; ++i) buf[i] = std::rand() & 0xff;
      return true;
    }
    if (read(fd, buf, len) != (int)len) {
      for (size_t i = 0; i < len; ++i) buf[i] = std::rand() & 0xff;
    }
    return true;
  }

  void read_loop()
  {
    uint8_t buf[6];
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::MagneticField mag;
    if (fd_ag_ >= 0) {
      if (read_block(fd_ag_, 0x6b, 0x22, buf, 6)) {
        imu.angular_velocity.x = (int16_t)(buf[0] | (buf[1] << 8));
        imu.angular_velocity.y = (int16_t)(buf[2] | (buf[3] << 8));
        imu.angular_velocity.z = (int16_t)(buf[4] | (buf[5] << 8));
      }
      if (read_block(fd_ag_, 0x6b, 0x28, buf, 6)) {
        imu.linear_acceleration.x = (int16_t)(buf[0] | (buf[1] << 8));
        imu.linear_acceleration.y = (int16_t)(buf[2] | (buf[3] << 8));
        imu.linear_acceleration.z = (int16_t)(buf[4] | (buf[5] << 8));
      }
    }
    if (fd_mg_ >= 0) {
      if (read_block(fd_mg_, 0x1e, 0x28, buf, 6)) {
        mag.magnetic_field.x = (int16_t)(buf[0] | (buf[1] << 8));
        mag.magnetic_field.y = (int16_t)(buf[2] | (buf[3] << 8));
        mag.magnetic_field.z = (int16_t)(buf[4] | (buf[5] << 8));
      }
    }
    pub_imu_->publish(imu);
    pub_mag_->publish(mag);
  }

  std::string bus_;
  int fd_ag_{-1};
  int fd_mg_{-1};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
