#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <csignal>
#include <execinfo.h>
#include <cstdlib>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

void crash_handler(int sig) {
  fprintf(stderr, "\n=== CRASH: signal %d ===\n", sig);
  void *bt[50];
  int n = backtrace(bt, 50);
  backtrace_symbols_fd(bt, n, STDERR_FILENO);
  fprintf(stderr, "=== END BACKTRACE ===\n");
  _exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGABRT, crash_handler);
  signal(SIGSEGV, crash_handler);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

  EGOReplanFSM rebo_replan;

  try {
    rebo_replan.init(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "INIT CRASH: %s", e.what());
    return 1;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
