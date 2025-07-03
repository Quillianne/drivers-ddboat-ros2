#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <chrono>
#include "ros2_ddboat/srv/pmtk_cmd.hpp"

using namespace std::chrono_literals;

class GPSNode : public rclcpp::Node
{
public:
  GPSNode() : Node("gps_node"), serial_()
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyGPS0");
    int baud = this->declare_parameter<int>("baud", 9600);
    serial_.setPort(port_);
    serial_.setBaudrate(baud);
    auto timeout = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(timeout);
    try {
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open port %s: %s", port_.c_str(), e.what());
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    timer_ = this->create_wall_timer(200ms, std::bind(&GPSNode::timer_callback, this));
    pmtk_service_ = this->create_service<ros2_ddboat::srv::PmtkCmd>(
      "pmtk_cmd",
      std::bind(&GPSNode::handle_pmtk, this,
        std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    if (!serial_.isOpen()) {
      return;
    }
    std::string line = serial_.readline();
    if (line.rfind("$GPGLL", 0) == 0 || line.rfind("$GPRMC", 0) == 0) {
      auto msg = sensor_msgs::msg::NavSatFix();
      double lat = 0.0, lon = 0.0;
      bool ok = false;
      if (line.rfind("$GPGLL", 0) == 0) {
        ok = parse_gpgll(line, lat, lon);
      } else if (line.rfind("$GPRMC", 0) == 0) {
        ok = parse_gprmc(line, lat, lon);
      }
      if (ok) {
        msg.latitude = lat;
        msg.longitude = lon;
        msg.header.stamp = now();
      }
      publisher_->publish(msg);
    }
  }

  static bool parse_gpgll(const std::string &line, double &lat, double &lon)
  {
    // $GPGLL,4916.45,N,12311.12,W,225444,A*1D
    auto parts = split(line, ',');
    if (parts.size() < 5) {
      return false;
    }
    lat = parse_ddmm(parts[1]);
    if (parts[2] == "S") lat = -lat;
    lon = parse_ddmm(parts[3]);
    if (parts[4] == "W") lon = -lon;
    return true;
  }

  static bool parse_gprmc(const std::string &line, double &lat, double &lon)
  {
    // $GPRMC,hhmmss,A,llll.ll,a,yyyyy.yy,a,...
    auto parts = split(line, ',');
    if (parts.size() < 7 || parts[2] != "A") {
      return false;
    }
    lat = parse_ddmm(parts[3]);
    if (parts[4] == "S") lat = -lat;
    lon = parse_ddmm(parts[5]);
    if (parts[6] == "W") lon = -lon;
    return true;
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

  static double parse_ddmm(const std::string &val)
  {
    if (val.size() < 4) return 0.0;
    double d = std::stod(val);
    int deg = static_cast<int>(d / 100);
    double minutes = d - deg * 100;
    return deg + minutes / 60.0;
  }

  void handle_pmtk(const std::shared_ptr<ros2_ddboat::srv::PmtkCmd::Request> req,
                   std::shared_ptr<ros2_ddboat::srv::PmtkCmd::Response> res)
  {
    if (!serial_.isOpen()) {
      res->success = false;
      return;
    }
    std::string cmd = req->command;
    if (cmd.find('\n') == std::string::npos) cmd += "\r\n";
    serial_.write(cmd);
    std::string reply = serial_.readline();
    res->response = reply;
    res->success = !reply.empty();
  }

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<ros2_ddboat::srv::PmtkCmd>::SharedPtr pmtk_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNode>());
  rclcpp::shutdown();
  return 0;
}
