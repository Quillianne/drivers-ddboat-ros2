#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class GPSNode : public rclcpp::Node
    srv_set_filter_ = this->create_service<std_srvs::srv::SetBool>(
      "set_filter_speed",
      std::bind(&GPSNode::handle_set_filter, this, std::placeholders::_1, std::placeholders::_2));
    srv_get_filter_ = this->create_service<std_srvs::srv::Trigger>(
      "get_filter_speed",
      std::bind(&GPSNode::handle_get_filter, this, std::placeholders::_1, std::placeholders::_2));
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
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_filter_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_get_filter_;

  void send_pmtk_cmd(const std::string& pmtk) {
    std::string msg = "$" + pmtk + "*" + compute_checksum(pmtk) + "\r\n";
    serial_.write(msg);
    RCLCPP_INFO(this->get_logger(), "Sent PMTK: %s", msg.c_str());
  }

  std::string compute_checksum(const std::string& str) {
    uint8_t csk = 0;
    for (char c : str) csk ^= c;
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", csk);
    return std::string(buf);
  }

  std::string wait_pmtk_response() {
    for (int i = 0; i < 20; ++i) {
      std::string line = serial_.readline();
      if (line.find("$PMTK") == 0) return line;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return "";
  }

  void handle_set_filter(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    // Utilise req->data comme valeur booléenne (0.0 ou 0.4 par exemple)
    double value = req->data ? 0.4 : 0.0;
    std::string pmtk = "PMTK386," + std::to_string(value);
    send_pmtk_cmd(pmtk);
    std::string resp = wait_pmtk_response();
    res->success = !resp.empty();
    res->message = resp;
  }

  void handle_get_filter(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    send_pmtk_cmd("PMTK447");
    std::string resp = wait_pmtk_response();
    res->success = !resp.empty();
    res->message = resp;
  }
  void timer_callback()
  {
    if (!serial_.isOpen()) {
      return;
    }
    std::string line = serial_.readline();
    if (line.empty()) return;
    RCLCPP_INFO(this->get_logger(), "NMEA: %s", line.c_str());
    double lat = 0.0, lon = 0.0;
    bool ok = false;
    if (line.rfind("$GPGLL", 0) == 0) {
      ok = parse_gpgll(line, lat, lon);
    } else if (line.rfind("$GPRMC", 0) == 0) {
      ok = parse_gprmc(line, lat, lon);
    }
    if (ok && lat != 0.0 && lon != 0.0) {
      auto msg = sensor_msgs::msg::NavSatFix();
      msg.latitude = lat;
      msg.longitude = lon;
      msg.header.stamp = now();
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published position: %.6f, %.6f", lat, lon);
    }
  }

  static bool parse_gpgll(const std::string &line, double &lat, double &lon)
  {
  // $GPGLL,4916.45,N,12311.12,W,225444,A*1D
  auto parts = split(line, ',');
  if (parts.size() < 7) return false;
  if (parts[6].empty() || parts[6][0] != 'A') return false; // statut doit être 'A'
  if (parts[1].empty() || parts[3].empty()) return false;
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
  if (parts.size() < 7 || parts[2] != "A") return false;
  if (parts[3].empty() || parts[5].empty()) return false;
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

  serial::Serial serial_;
  std::string port_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNode>());
  rclcpp::shutdown();
  return 0;
}
