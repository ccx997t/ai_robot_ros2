#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
class BaseStatusNode : public rclcpp::Node {
public:
  BaseStatusNode() : Node("base_status_node") {
    mode_ = declare_parameter<std::string>("mode", "sim");
    timer_ = create_wall_timer(1s, [this]() { RCLCPP_INFO(get_logger(), "base foundation active (mode=%s); motor output is disabled", mode_.c_str()); });
  }
private:
  std::string mode_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char * argv[]) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<BaseStatusNode>()); rclcpp::shutdown(); return 0; }
