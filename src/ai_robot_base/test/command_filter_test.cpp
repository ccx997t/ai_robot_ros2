#include <cmath>
#include <cstdlib>

#include "ai_robot_base/command_filter.hpp"

int main() {
  const ai_robot_base::CommandFilter filter(0.30, 0.80);
  ai_robot_base::PlanarCommand output{};
  if (!filter.filter({0.10, -0.20}, output) || output.linear_x != 0.10 || output.angular_z != -0.20) {
    return EXIT_FAILURE;
  }
  if (!filter.filter({1.0, -2.0}, output) || output.linear_x != 0.30 || output.angular_z != -0.80) {
    return EXIT_FAILURE;
  }
  if (filter.filter({NAN, 0.0}, output) || output.linear_x != 0.0 || output.angular_z != 0.0) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
