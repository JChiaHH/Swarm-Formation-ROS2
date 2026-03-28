#include <rclcpp/rclcpp.hpp>
#include <so3_control/so3_control_component.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<so3_control::SO3ControlComponent>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
