#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class OdomContractRelay : public rclcpp::Node {
public:
  OdomContractRelay() : Node("odom_contract_relay") {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/base_controller/odom");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/odom");
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        publisher_->publish(*message);
      });
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomContractRelay>());
  rclcpp::shutdown();
  return 0;
}
