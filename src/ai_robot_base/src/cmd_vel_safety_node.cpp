#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ai_robot_base/command_filter.hpp"
#include "ai_robot_base/command_watchdog.hpp"

using namespace std::chrono_literals;

class CmdVelSafetyNode : public rclcpp::Node {
public:
  CmdVelSafetyNode()
  : Node("cmd_vel_safety"),
    timeout_(validate_parameter(
      declare_parameter<double>("command_timeout_seconds", 0.5), 2.0,
      "command_timeout_seconds")),
    filter_(validate_parameter(
      declare_parameter<double>("max_linear_speed_mps", 0.30), 1.0,
      "max_linear_speed_mps"),
      validate_parameter(
      declare_parameter<double>("max_angular_speed_rps", 0.80), 3.0,
      "max_angular_speed_rps")),
    last_command_(std::chrono::steady_clock::now())
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/base_controller/cmd_vel_unstamped");
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(1).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { receive_command(*msg); });
    timer_ = create_wall_timer(50ms, [this]() { check_timeout(); });
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "waiting for command");
  }

  ~CmdVelSafetyNode() override { publish_stop(); }

private:
  static double validate_parameter(double value, double upper, const char * name) {
    if (!std::isfinite(value) || value <= 0.0 || value > upper) {
      throw std::invalid_argument(std::string(name) + " is outside its safe range");
    }
    return value;
  }

  void receive_command(const geometry_msgs::msg::Twist & input) {
    ai_robot_base::PlanarCommand filtered{};
    const bool valid = filter_.filter({input.linear.x, input.angular.z}, filtered);
    geometry_msgs::msg::Twist output;
    output.linear.x = filtered.linear_x;
    output.angular.z = filtered.angular_z;
    command_pub_->publish(output);
    last_command_ = std::chrono::steady_clock::now();
    timed_out_ = false;
    if (!valid) {
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                         "non-finite command rejected; stop sent");
      return;
    }
    const bool limited = filtered.linear_x != input.linear.x || filtered.angular_z != input.angular.z;
    publish_diagnostic(
      limited ? diagnostic_msgs::msg::DiagnosticStatus::WARN
              : diagnostic_msgs::msg::DiagnosticStatus::OK,
      limited ? "command limited" : "command accepted");
  }

  void check_timeout() {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_).count();
    if (timeout_.is_expired(elapsed)) {
      publish_stop();
      if (!timed_out_) {
        timed_out_ = true;
        publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           "command timeout; stop sent");
      }
    }
  }

  void publish_stop() {
    if (command_pub_) command_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_diagnostic(uint8_t level, const std::string & message) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "base/cmd_vel_safety";
    status.hardware_id = "simulation";
    status.message = message;
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  ai_robot_base::CommandWatchdog timeout_;
  ai_robot_base::CommandFilter filter_;
  std::string output_topic_;
  std::chrono::steady_clock::time_point last_command_;
  bool timed_out_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
