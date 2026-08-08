#ifndef AI_ROBOT_BASE__COMMAND_WATCHDOG_HPP_
#define AI_ROBOT_BASE__COMMAND_WATCHDOG_HPP_
namespace ai_robot_base {
class CommandWatchdog {
public:
  explicit CommandWatchdog(double timeout_seconds) : timeout_seconds_(timeout_seconds) {}
  bool is_expired(double elapsed_seconds) const { return elapsed_seconds >= timeout_seconds_; }
private:
  double timeout_seconds_;
};
}  // namespace ai_robot_base
#endif  // AI_ROBOT_BASE__COMMAND_WATCHDOG_HPP_
