#ifndef AI_ROBOT_BASE__COMMAND_FILTER_HPP_
#define AI_ROBOT_BASE__COMMAND_FILTER_HPP_

#include <algorithm>
#include <cmath>

namespace ai_robot_base {

struct PlanarCommand {
  double linear_x;
  double angular_z;
};

class CommandFilter {
public:
  CommandFilter(double max_linear, double max_angular)
  : max_linear_(std::abs(max_linear)), max_angular_(std::abs(max_angular)) {}

  bool filter(const PlanarCommand & input, PlanarCommand & output) const {
    if (!std::isfinite(input.linear_x) || !std::isfinite(input.angular_z)) {
      output = {0.0, 0.0};
      return false;
    }
    output.linear_x = std::clamp(input.linear_x, -max_linear_, max_linear_);
    output.angular_z = std::clamp(input.angular_z, -max_angular_, max_angular_);
    return true;
  }

private:
  double max_linear_;
  double max_angular_;
};

}  // namespace ai_robot_base
#endif  // AI_ROBOT_BASE__COMMAND_FILTER_HPP_
