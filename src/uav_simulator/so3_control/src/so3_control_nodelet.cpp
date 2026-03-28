#include <so3_control/so3_control_component.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace so3_control
{

SO3ControlComponent::SO3ControlComponent(const rclcpp::NodeOptions& options)
  : Node("so3_control", options)
  , position_cmd_updated_(false)
  , position_cmd_init_(false)
  , des_yaw_(0)
  , des_yaw_dot_(0)
  , current_yaw_(0)
  , enable_motors_(true)  // FIXME
  , use_external_yaw_(false)
{
  // Declare and get parameters
  if (!this->has_parameter("quadrotor_name")) {
    this->declare_parameter<std::string>("quadrotor_name", "quadrotor");
  }
  std::string quadrotor_name = this->get_parameter("quadrotor_name").as_string();
  frame_id_ = "/" + quadrotor_name;

  if (!this->has_parameter("mass")) {
    this->declare_parameter<double>("mass", 0.5);
  }
  double mass = this->get_parameter("mass").as_double();
  controller_.setMass(mass);

  if (!this->has_parameter("use_external_yaw")) {
    this->declare_parameter<bool>("use_external_yaw", true);
  }
  use_external_yaw_ = this->get_parameter("use_external_yaw").as_bool();

  if (!this->has_parameter("gains.rot.x")) {
    this->declare_parameter<double>("gains.rot.x", 1.5);
  }
  if (!this->has_parameter("gains.rot.y")) {
    this->declare_parameter<double>("gains.rot.y", 1.5);
  }
  if (!this->has_parameter("gains.rot.z")) {
    this->declare_parameter<double>("gains.rot.z", 1.0);
  }
  if (!this->has_parameter("gains.ang.x")) {
    this->declare_parameter<double>("gains.ang.x", 0.13);
  }
  if (!this->has_parameter("gains.ang.y")) {
    this->declare_parameter<double>("gains.ang.y", 0.13);
  }
  if (!this->has_parameter("gains.ang.z")) {
    this->declare_parameter<double>("gains.ang.z", 0.1);
  }
  if (!this->has_parameter("gains.kx.x")) {
    this->declare_parameter<double>("gains.kx.x", 5.7);
  }
  if (!this->has_parameter("gains.kx.y")) {
    this->declare_parameter<double>("gains.kx.y", 5.7);
  }
  if (!this->has_parameter("gains.kx.z")) {
    this->declare_parameter<double>("gains.kx.z", 6.2);
  }
  if (!this->has_parameter("gains.kv.x")) {
    this->declare_parameter<double>("gains.kv.x", 3.4);
  }
  if (!this->has_parameter("gains.kv.y")) {
    this->declare_parameter<double>("gains.kv.y", 3.4);
  }
  if (!this->has_parameter("gains.kv.z")) {
    this->declare_parameter<double>("gains.kv.z", 4.0);
  }

  kR_[0] = this->get_parameter("gains.rot.x").as_double();
  kR_[1] = this->get_parameter("gains.rot.y").as_double();
  kR_[2] = this->get_parameter("gains.rot.z").as_double();
  kOm_[0] = this->get_parameter("gains.ang.x").as_double();
  kOm_[1] = this->get_parameter("gains.ang.y").as_double();
  kOm_[2] = this->get_parameter("gains.ang.z").as_double();
  kx_[0] = this->get_parameter("gains.kx.x").as_double();
  kx_[1] = this->get_parameter("gains.kx.y").as_double();
  kx_[2] = this->get_parameter("gains.kx.z").as_double();
  kv_[0] = this->get_parameter("gains.kv.x").as_double();
  kv_[1] = this->get_parameter("gains.kv.y").as_double();
  kv_[2] = this->get_parameter("gains.kv.z").as_double();

  if (!this->has_parameter("corrections.z")) {
    this->declare_parameter<double>("corrections.z", 0.0);
  }
  if (!this->has_parameter("corrections.r")) {
    this->declare_parameter<double>("corrections.r", 0.0);
  }
  if (!this->has_parameter("corrections.p")) {
    this->declare_parameter<double>("corrections.p", 0.0);
  }
  corrections_[0] = this->get_parameter("corrections.z").as_double();
  corrections_[1] = this->get_parameter("corrections.r").as_double();
  corrections_[2] = this->get_parameter("corrections.p").as_double();

  if (!this->has_parameter("so3_control.init_state_x")) {
    this->declare_parameter<double>("so3_control.init_state_x", 0.0);
  }
  if (!this->has_parameter("so3_control.init_state_y")) {
    this->declare_parameter<double>("so3_control.init_state_y", 0.0);
  }
  if (!this->has_parameter("so3_control.init_state_z")) {
    this->declare_parameter<double>("so3_control.init_state_z", -10000.0);
  }
  init_x_ = this->get_parameter("so3_control.init_state_x").as_double();
  init_y_ = this->get_parameter("so3_control.init_state_y").as_double();
  init_z_ = this->get_parameter("so3_control.init_state_z").as_double();

  // Publishers
  so3_command_pub_ = this->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&SO3ControlComponent::odom_callback, this, std::placeholders::_1));

  position_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
    "position_cmd", 10,
    std::bind(&SO3ControlComponent::position_cmd_callback, this, std::placeholders::_1));

  enable_motors_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "motors", 2,
    std::bind(&SO3ControlComponent::enable_motors_callback, this, std::placeholders::_1));

  corrections_sub_ = this->create_subscription<quadrotor_msgs::msg::Corrections>(
    "corrections", 10,
    std::bind(&SO3ControlComponent::corrections_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10,
    std::bind(&SO3ControlComponent::imu_callback, this, std::placeholders::_1));
}

void
SO3ControlComponent::publishSO3Command(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  auto so3_command = quadrotor_msgs::msg::SO3Command();
  so3_command.header.stamp    = this->now();
  so3_command.header.frame_id = frame_id_;
  so3_command.force.x         = force(0);
  so3_command.force.y         = force(1);
  so3_command.force.z         = force(2);
  so3_command.orientation.x   = orientation.x();
  so3_command.orientation.y   = orientation.y();
  so3_command.orientation.z   = orientation.z();
  so3_command.orientation.w   = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    so3_command.k_r[i]  = kR_[i];
    so3_command.k_om[i] = kOm_[i];
  }
  so3_command.aux.current_yaw          = current_yaw_;
  so3_command.aux.kf_correction        = corrections_[0];
  so3_command.aux.angle_corrections[0] = corrections_[1];
  so3_command.aux.angle_corrections[1] = corrections_[2];
  so3_command.aux.enable_motors        = enable_motors_;
  so3_command.aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_->publish(so3_command);
}

void
SO3ControlComponent::position_cmd_callback(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  if ( cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5 )
  {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if ( cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5 )
  {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();
}

void
SO3ControlComponent::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);
  current_yaw_ = tf2::getYaw(q);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)
      publishSO3Command();
    position_cmd_updated_ = false;
  }
  else if ( init_z_ > -9999.0 )
  {
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0,0,0);
    des_acc_ = Eigen::Vector3d(0,0,0);
    publishSO3Command();
  }
}

void
SO3ControlComponent::enable_motors_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
    RCLCPP_INFO(this->get_logger(), "Enabling motors");
  else
    RCLCPP_INFO(this->get_logger(), "Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlComponent::corrections_callback(
  const quadrotor_msgs::msg::Corrections::SharedPtr msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlComponent::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  const Eigen::Vector3d acc(imu->linear_acceleration.x,
                            imu->linear_acceleration.y,
                            imu->linear_acceleration.z);
  controller_.setAcc(acc);
}

}  // namespace so3_control

RCLCPP_COMPONENTS_REGISTER_NODE(so3_control::SO3ControlComponent)
