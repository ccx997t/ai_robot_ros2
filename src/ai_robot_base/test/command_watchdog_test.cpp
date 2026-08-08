#include <cstdlib>
#include "ai_robot_base/command_watchdog.hpp"
int main() {
  const ai_robot_base::CommandWatchdog watchdog(0.5);
  return (watchdog.is_expired(0.49) || !watchdog.is_expired(0.5) || !watchdog.is_expired(1.0)) ? EXIT_FAILURE : EXIT_SUCCESS;
}
