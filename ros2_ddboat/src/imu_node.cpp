#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <limits>

static constexpr double PI = 3.14159265358979323846;
#include <string>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node
{
public:
  ImuNode() : Node("imu_node")
  {
    bus_ = this->declare_parameter<std::string>("bus", "/dev/i2c-1");
    open_bus();
    configure_sensors();
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("imu/heading", 10);
    srv_calib_ = this->create_service<std_srvs::srv::Trigger>(
      "fast_heading_calibration",
      std::bind(&ImuNode::calib_service, this, std::placeholders::_1, std::placeholders::_2));
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

  bool write_reg(int fd, uint8_t addr, uint8_t reg, uint8_t val)
  {
    uint8_t buf[2] = {reg, val};
    if (ioctl(fd, I2C_SLAVE, addr) < 0) return false;
    if (write(fd, buf, 2) != 2) return false;
    return true;
  }

  void configure_sensors()
  {
    if (fd_ag_ >= 0) {
      // Accelerometer: 119 Hz, +-4g, enable XYZ (CTRL_REG6_XL and CTRL_REG5_XL)
      write_reg(fd_ag_, 0x6b, 0x20, 0x50); // CTRL_REG6_XL
      write_reg(fd_ag_, 0x6b, 0x1F, 0x38); // CTRL_REG5_XL
    }
    if (fd_mg_ >= 0) {
      // Magnetometer configuration for continuous mode
      write_reg(fd_mg_, 0x1e, 0x20, 0x70); // CTRL_REG1_M: temp comp, high perf, 80Hz
      write_reg(fd_mg_, 0x1e, 0x21, 0x00); // CTRL_REG2_M: +/-4 gauss
      write_reg(fd_mg_, 0x1e, 0x22, 0x00); // CTRL_REG3_M: continuous conversion
      write_reg(fd_mg_, 0x1e, 0x23, 0x0c); // CTRL_REG4_M: Z high performance
    }
  }

  bool read_block(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
  {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) return false;
    if (write(fd, &reg, 1) != 1) return false;
    if (read(fd, buf, len) != (int)len) return false;
    return true;
  }

  void calib_service(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                     std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (!std::isnan(last_mag_x_)) {
      heading_offset_ = std::atan2(last_mag_y_, last_mag_x_);
      has_heading_offset_ = true;
      res->success = true;
      res->message = "Heading calibrated";
    } else {
      res->success = false;
      res->message = "No magnetometer data";
    }
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
        last_mag_x_ = mag.magnetic_field.x;
        last_mag_y_ = mag.magnetic_field.y;
        last_mag_z_ = mag.magnetic_field.z;
      }
    }
    pub_imu_->publish(imu);
    pub_mag_->publish(mag);

    if (has_heading_offset_) {
      double heading = std::atan2(mag.magnetic_field.y, mag.magnetic_field.x) - heading_offset_;
      if (heading > PI) heading -= 2*PI;
      if (heading < -PI) heading += 2*PI;
      std_msgs::msg::Float64 msg;
      msg.data = heading;
      pub_heading_->publish(msg);
    }
  }

  std::string bus_;
  int fd_ag_{-1};
  int fd_mg_{-1};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_calib_;
  rclcpp::TimerBase::SharedPtr timer_;

  double last_mag_x_{std::numeric_limits<double>::quiet_NaN()};
  double last_mag_y_{std::numeric_limits<double>::quiet_NaN()};
  double last_mag_z_{std::numeric_limits<double>::quiet_NaN()};
  double heading_offset_{0.0};
  bool has_heading_offset_{false};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
