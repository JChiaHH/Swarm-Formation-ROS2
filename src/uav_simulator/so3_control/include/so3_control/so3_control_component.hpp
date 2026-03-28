#ifndef SO3_CONTROL__SO3_CONTROL_COMPONENT_HPP_
#define SO3_CONTROL__SO3_CONTROL_COMPONENT_HPP_

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.h>
#include <std_msgs/msg/bool.hpp>

namespace so3_control
{

class SO3ControlComponent : public rclcpp::Node
{
public:
  explicit SO3ControlComponent(const rclcpp::NodeOptions& options);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(
    const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void enable_motors_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void corrections_callback(const quadrotor_msgs::msg::Corrections::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);

  SO3Control controller_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool        position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double          des_yaw_, des_yaw_dot_;
  double          current_yaw_;
  bool            enable_motors_;
  bool            use_external_yaw_;
  double          kR_[3], kOm_[3], corrections_[3];
  double          init_x_, init_y_, init_z_;
};

}  // namespace so3_control

#endif  // SO3_CONTROL__SO3_CONTROL_COMPONENT_HPP_
