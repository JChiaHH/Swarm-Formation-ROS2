#include <iostream>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "armadillo"
#include "pose_utils.h"
#include <quadrotor_msgs/msg/position_command.hpp>

using namespace arma;
using namespace std;
using std::placeholders::_1;

class OdomVisualization : public rclcpp::Node
{
public:
  OdomVisualization() : Node("odom_visualization")
  {
    // Declare parameters
    if (!this->has_parameter("mesh_resource")) {
      this->declare_parameter<std::string>("mesh_resource", "package://odom_visualization/meshes/hummingbird.mesh");
    }
    if (!this->has_parameter("color/r")) {
      this->declare_parameter<double>("color/r", 1.0);
    }
    if (!this->has_parameter("color/g")) {
      this->declare_parameter<double>("color/g", 0.0);
    }
    if (!this->has_parameter("color/b")) {
      this->declare_parameter<double>("color/b", 0.0);
    }
    if (!this->has_parameter("color/a")) {
      this->declare_parameter<double>("color/a", 1.0);
    }
    if (!this->has_parameter("origin")) {
      this->declare_parameter<bool>("origin", false);
    }
    if (!this->has_parameter("robot_scale")) {
      this->declare_parameter<double>("robot_scale", 2.0);
    }
    if (!this->has_parameter("frame_id")) {
      this->declare_parameter<std::string>("frame_id", "world");
    }
    if (!this->has_parameter("cross_config")) {
      this->declare_parameter<bool>("cross_config", false);
    }
    if (!this->has_parameter("tf45")) {
      this->declare_parameter<bool>("tf45", false);
    }
    if (!this->has_parameter("covariance_scale")) {
      this->declare_parameter<double>("covariance_scale", 100.0);
    }
    if (!this->has_parameter("covariance_position")) {
      this->declare_parameter<bool>("covariance_position", false);
    }
    if (!this->has_parameter("covariance_velocity")) {
      this->declare_parameter<bool>("covariance_velocity", false);
    }
    if (!this->has_parameter("covariance_color")) {
      this->declare_parameter<bool>("covariance_color", false);
    }
    if (!this->has_parameter("drone_id")) {
      this->declare_parameter<int>("drone_id", -1);
    }

    // Get parameters
    this->get_parameter("mesh_resource", mesh_resource_);
    this->get_parameter("color/r", color_r_);
    this->get_parameter("color/g", color_g_);
    this->get_parameter("color/b", color_b_);
    this->get_parameter("color/a", color_a_);
    this->get_parameter("origin", origin_);
    this->get_parameter("robot_scale", scale_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("cross_config", cross_config_);
    this->get_parameter("tf45", tf45_);
    this->get_parameter("covariance_scale", cov_scale_);
    this->get_parameter("covariance_position", cov_pos_);
    this->get_parameter("covariance_velocity", cov_vel_);
    this->get_parameter("covariance_color", cov_color_);
    this->get_parameter("drone_id", drone_id_);

    // Publishers
    pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
    path_pub_    = this->create_publisher<nav_msgs::msg::Path>("path", 100);
    vel_pub_     = this->create_publisher<visualization_msgs::msg::Marker>("velocity", 100);
    cov_pub_     = this->create_publisher<visualization_msgs::msg::Marker>("covariance", 100);
    cov_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("covariance_velocity", 100);
    traj_pub_    = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", 100);
    sensor_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("sensor", 100);
    mesh_pub_    = this->create_publisher<visualization_msgs::msg::Marker>("robot", 100);
    height_pub_  = this->create_publisher<sensor_msgs::msg::Range>("height", 100);

    // TF broadcaster
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 100, std::bind(&OdomVisualization::odom_callback, this, _1));
    sub_cmd_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "cmd", 100, std::bind(&OdomVisualization::cmd_callback, this, _1));

    // Initialize state
    poseOrigin_.resize(6);
    is_origin_set_ = false;
    prevt_initialized_ = false;
    pt_initialized_ = false;
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id == string("null"))
      return;

    colvec pose(6);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    colvec q(4);

    q(0)    = msg->pose.pose.orientation.w;
    q(1)    = msg->pose.pose.orientation.x;
    q(2)    = msg->pose.pose.orientation.y;
    q(3)    = msg->pose.pose.orientation.z;
    pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
    colvec vel(3);

    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    if (origin_ && !is_origin_set_)
    {
      is_origin_set_ = true;
      poseOrigin_  = pose;
    }
    if (origin_)
    {
      vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;
      pose = pose_update(pose_inverse(poseOrigin_), pose);
      vel  = ypr_to_R(pose.rows(3,5)) * vel;
    }

    // Pose
    geometry_msgs::msg::PoseStamped poseROS;
    poseROS.header = msg->header;
    poseROS.header.stamp = msg->header.stamp;
    poseROS.header.frame_id = string("world");
    poseROS.pose.position.x = pose(0);
    poseROS.pose.position.y = pose(1);
    poseROS.pose.position.z = pose(2);
    q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
    poseROS.pose.orientation.w = q(0);
    poseROS.pose.orientation.x = q(1);
    poseROS.pose.orientation.y = q(2);
    poseROS.pose.orientation.z = q(3);
    pose_pub_->publish(poseROS);

    // Velocity
    colvec yprVel(3);
    yprVel(0) =  atan2(vel(1), vel(0));
    yprVel(1) = -atan2(vel(2), norm(vel.rows(0,1),2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    visualization_msgs::msg::Marker velROS;
    velROS.header.frame_id = string("world");
    velROS.header.stamp = msg->header.stamp;
    velROS.ns = string("velocity");
    velROS.id = 0;
    velROS.type = visualization_msgs::msg::Marker::ARROW;
    velROS.action = visualization_msgs::msg::Marker::ADD;
    velROS.pose.position.x = pose(0);
    velROS.pose.position.y = pose(1);
    velROS.pose.position.z = pose(2);
    velROS.pose.orientation.w = q(0);
    velROS.pose.orientation.x = q(1);
    velROS.pose.orientation.y = q(2);
    velROS.pose.orientation.z = q(3);
    velROS.scale.x = norm(vel, 2);
    velROS.scale.y = 0.05;
    velROS.scale.z = 0.05;
    velROS.color.a = 1.0;
    velROS.color.r = color_r_;
    velROS.color.g = color_g_;
    velROS.color.b = color_b_;
    vel_pub_->publish(velROS);

    // Path
    rclcpp::Time current_stamp(msg->header.stamp);
    if (!prevt_initialized_)
    {
      prevt_ = current_stamp;
      prevt_initialized_ = true;
    }
    if ((current_stamp - prevt_).seconds() > 0.1)
    {
      prevt_ = current_stamp;
      path_ros_.header = poseROS.header;
      path_ros_.poses.push_back(poseROS);
      path_pub_->publish(path_ros_);
    }

    // Covariance color
    double r = 1;
    double g = 1;
    double b = 1;
    bool G = msg->twist.covariance[33];
    bool V = msg->twist.covariance[34];
    bool L = msg->twist.covariance[35];
    if (cov_color_)
    {
      r = G;
      g = V;
      b = L;
    }

    // Covariance Position
    if (cov_pos_)
    {
      mat P(6,6);
      for (int j = 0; j < 6; j++)
        for (int i = 0; i < 6; i++)
          P(i,j) = msg->pose.covariance[i+j*6];
      colvec eigVal;
      mat    eigVec;
      eig_sym(eigVal, eigVec, P.submat(0,0,2,2));
      if (det(eigVec) < 0)
      {
        for (int k = 0; k < 3; k++)
        {
          mat eigVecRev = eigVec;
          eigVecRev.col(k) *= -1;
          if (det(eigVecRev) > 0)
          {
            eigVec = eigVecRev;
            break;
          }
        }
      }
      visualization_msgs::msg::Marker covROS;
      covROS.header.frame_id = string("world");
      covROS.header.stamp = msg->header.stamp;
      covROS.ns = string("covariance");
      covROS.id = 0;
      covROS.type = visualization_msgs::msg::Marker::SPHERE;
      covROS.action = visualization_msgs::msg::Marker::ADD;
      covROS.pose.position.x = pose(0);
      covROS.pose.position.y = pose(1);
      covROS.pose.position.z = pose(2);
      q = R_to_quaternion(eigVec);
      covROS.pose.orientation.w = q(0);
      covROS.pose.orientation.x = q(1);
      covROS.pose.orientation.y = q(2);
      covROS.pose.orientation.z = q(3);
      covROS.scale.x = sqrt(eigVal(0))*cov_scale_;
      covROS.scale.y = sqrt(eigVal(1))*cov_scale_;
      covROS.scale.z = sqrt(eigVal(2))*cov_scale_;
      covROS.color.a = 0.4;
      covROS.color.r = r * 0.5;
      covROS.color.g = g * 0.5;
      covROS.color.b = b * 0.5;
      cov_pub_->publish(covROS);
    }

    // Covariance Velocity
    if (cov_vel_)
    {
      mat P(3,3);
      for (int j = 0; j < 3; j++)
        for (int i = 0; i < 3; i++)
          P(i,j) = msg->twist.covariance[i+j*6];
      mat R = ypr_to_R(pose.rows(3,5));
      P = R * P * trans(R);
      colvec eigVal;
      mat    eigVec;
      eig_sym(eigVal, eigVec, P);
      if (det(eigVec) < 0)
      {
        for (int k = 0; k < 3; k++)
        {
          mat eigVecRev = eigVec;
          eigVecRev.col(k) *= -1;
          if (det(eigVecRev) > 0)
          {
            eigVec = eigVecRev;
            break;
          }
        }
      }
      visualization_msgs::msg::Marker covVelROS;
      covVelROS.header.frame_id = string("world");
      covVelROS.header.stamp = msg->header.stamp;
      covVelROS.ns = string("covariance_velocity");
      covVelROS.id = 0;
      covVelROS.type = visualization_msgs::msg::Marker::SPHERE;
      covVelROS.action = visualization_msgs::msg::Marker::ADD;
      covVelROS.pose.position.x = pose(0);
      covVelROS.pose.position.y = pose(1);
      covVelROS.pose.position.z = pose(2);
      q = R_to_quaternion(eigVec);
      covVelROS.pose.orientation.w = q(0);
      covVelROS.pose.orientation.x = q(1);
      covVelROS.pose.orientation.y = q(2);
      covVelROS.pose.orientation.z = q(3);
      covVelROS.scale.x = sqrt(eigVal(0))*cov_scale_;
      covVelROS.scale.y = sqrt(eigVal(1))*cov_scale_;
      covVelROS.scale.z = sqrt(eigVal(2))*cov_scale_;
      covVelROS.color.a = 0.4;
      covVelROS.color.r = r;
      covVelROS.color.g = g;
      covVelROS.color.b = b;
      cov_vel_pub_->publish(covVelROS);
    }

    // Color Coded Trajectory
    rclcpp::Time t(msg->header.stamp);
    if (!pt_initialized_)
    {
      ppose_ = pose;
      pt_ = t;
      pt_initialized_ = true;
    }
    if ((t - pt_).seconds() > 0.5)
    {
      visualization_msgs::msg::Marker trajROS;
      trajROS.header.frame_id = string("world");
      trajROS.header.stamp    = this->now();
      trajROS.ns              = string("trajectory");
      trajROS.type            = visualization_msgs::msg::Marker::LINE_LIST;
      trajROS.action          = visualization_msgs::msg::Marker::ADD;
      trajROS.pose.position.x    = 0;
      trajROS.pose.position.y    = 0;
      trajROS.pose.position.z    = 0;
      trajROS.pose.orientation.w = 1;
      trajROS.pose.orientation.x = 0;
      trajROS.pose.orientation.y = 0;
      trajROS.pose.orientation.z = 0;
      trajROS.scale.x = 0.1;
      trajROS.scale.y = 0;
      trajROS.scale.z = 0;
      trajROS.color.r = 0.0;
      trajROS.color.g = 1.0;
      trajROS.color.b = 0.0;
      trajROS.color.a = 0.8;
      geometry_msgs::msg::Point p;
      p.x = ppose_(0);
      p.y = ppose_(1);
      p.z = ppose_(2);
      trajROS.points.push_back(p);
      p.x = pose(0);
      p.y = pose(1);
      p.z = pose(2);
      trajROS.points.push_back(p);
      std_msgs::msg::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = 1;
      trajROS.colors.push_back(color);
      trajROS.colors.push_back(color);
      ppose_ = pose;
      pt_ = t;
      traj_pub_->publish(trajROS);
    }

    // Sensor availability
    visualization_msgs::msg::Marker sensorROS;
    sensorROS.header.frame_id = string("world");
    sensorROS.header.stamp    = msg->header.stamp;
    sensorROS.ns              = string("sensor");
    sensorROS.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    sensorROS.action          = visualization_msgs::msg::Marker::ADD;
    sensorROS.pose.position.x = pose(0);
    sensorROS.pose.position.y = pose(1);
    sensorROS.pose.position.z = pose(2) + 1.0;
    sensorROS.pose.orientation.w = q(0);
    sensorROS.pose.orientation.x = q(1);
    sensorROS.pose.orientation.y = q(2);
    sensorROS.pose.orientation.z = q(3);
    string strG = G?string(" GPS "):string("");
    string strV = V?string(" Vision "):string("");
    string strL = L?string(" Laser "):string("");
    sensorROS.text = "| " + strG + strV + strL + " |";
    sensorROS.color.a = 1.0;
    sensorROS.color.r = 1.0;
    sensorROS.color.g = 1.0;
    sensorROS.color.b = 1.0;
    sensorROS.scale.z = 0.5;
    sensor_pub_->publish(sensorROS);

    // Laser height measurement
    double H = msg->twist.covariance[32];
    sensor_msgs::msg::Range heightROS;
    heightROS.header.frame_id = string("height");
    heightROS.header.stamp = msg->header.stamp;
    heightROS.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    heightROS.field_of_view = 5.0 * M_PI / 180.0;
    heightROS.min_range = -100;
    heightROS.max_range =  100;
    heightROS.range     =  H;
    height_pub_->publish(heightROS);

    // Mesh model
    visualization_msgs::msg::Marker meshROS;
    meshROS.header.frame_id = frame_id_;
    meshROS.header.stamp = msg->header.stamp;
    meshROS.ns = "mesh";
    meshROS.id = drone_id_;
    meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::msg::Marker::ADD;
    meshROS.pose.position.x = msg->pose.pose.position.x;
    meshROS.pose.position.y = msg->pose.pose.position.y;
    meshROS.pose.position.z = msg->pose.pose.position.z;
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    if (cross_config_)
    {
      colvec ypr = R_to_ypr(quaternion_to_R(q));
      ypr(0)    += 45.0*PI/180.0;
      q          = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS.pose.orientation.w = q(0);
    meshROS.pose.orientation.x = q(1);
    meshROS.pose.orientation.y = q(2);
    meshROS.pose.orientation.z = q(3);
    meshROS.scale.x = scale_;
    meshROS.scale.y = scale_;
    meshROS.scale.z = scale_;
    meshROS.color.a = color_a_;
    meshROS.color.r = color_r_;
    meshROS.color.g = color_g_;
    meshROS.color.b = color_b_;
    meshROS.mesh_resource = mesh_resource_;
    mesh_pub_->publish(meshROS);

    // TF for raw sensor visualization
    if (tf45_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = string("world");
      string base_s   = drone_id_ == -1 ? string("base")   : string("base")  + std::to_string(drone_id_);
      string laser_s  = drone_id_ == -1 ? string("laser")  : string("laser") + std::to_string(drone_id_);
      string vision_s = drone_id_ == -1 ? string("vision") : string("vision")+ std::to_string(drone_id_);
      string height_s = drone_id_ == -1 ? string("height") : string("height")+ std::to_string(drone_id_);

      transform.child_frame_id = base_s;
      transform.transform.translation.x = pose(0);
      transform.transform.translation.y = pose(1);
      transform.transform.translation.z = pose(2);
      transform.transform.rotation.x = q(1);
      transform.transform.rotation.y = q(2);
      transform.transform.rotation.z = q(3);
      transform.transform.rotation.w = q(0);
      broadcaster_->sendTransform(transform);

      // 45-degree transform
      colvec y45 = zeros<colvec>(3);
      y45(0) = 45.0 * M_PI/180;
      colvec q45 = R_to_quaternion(ypr_to_R(y45));

      geometry_msgs::msg::TransformStamped transform45;
      transform45.header.stamp = msg->header.stamp;
      transform45.header.frame_id = base_s;
      transform45.transform.translation.x = 0;
      transform45.transform.translation.y = 0;
      transform45.transform.translation.z = 0;
      transform45.transform.rotation.x = q45(1);
      transform45.transform.rotation.y = q45(2);
      transform45.transform.rotation.z = q45(3);
      transform45.transform.rotation.w = q45(0);

      transform45.child_frame_id = laser_s;
      broadcaster_->sendTransform(transform45);

      transform45.child_frame_id = vision_s;
      broadcaster_->sendTransform(transform45);

      // 90-degree transform
      colvec p90 = zeros<colvec>(3);
      p90(1) = 90.0 * M_PI/180;
      colvec q90 = R_to_quaternion(ypr_to_R(p90));

      geometry_msgs::msg::TransformStamped transform90;
      transform90.header.stamp = msg->header.stamp;
      transform90.header.frame_id = base_s;
      transform90.child_frame_id = height_s;
      transform90.transform.translation.x = 0;
      transform90.transform.translation.y = 0;
      transform90.transform.translation.z = 0;
      transform90.transform.rotation.x = q90(1);
      transform90.transform.rotation.y = q90(2);
      transform90.transform.rotation.z = q90(3);
      transform90.transform.rotation.w = q90(0);
      broadcaster_->sendTransform(transform90);
    }
  }

  void cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
  {
    if (cmd->header.frame_id == string("null"))
      return;

    colvec pose(6);
    pose(0) = cmd->position.x;
    pose(1) = cmd->position.y;
    pose(2) = cmd->position.z;
    colvec q(4);
    q(0)    = 1.0;
    q(1)    = 0.0;
    q(2)    = 0.0;
    q(3)    = 0.0;
    pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));

    // Mesh model
    visualization_msgs::msg::Marker meshROS;
    meshROS.header.frame_id = frame_id_;
    meshROS.header.stamp = cmd->header.stamp;
    meshROS.ns = "mesh";
    meshROS.id = drone_id_;
    meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::msg::Marker::ADD;
    meshROS.pose.position.x = cmd->position.x;
    meshROS.pose.position.y = cmd->position.y;
    meshROS.pose.position.z = cmd->position.z;

    if (cross_config_)
    {
      colvec ypr = R_to_ypr(quaternion_to_R(q));
      ypr(0)    += 45.0*PI/180.0;
      q          = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS.pose.orientation.w = q(0);
    meshROS.pose.orientation.x = q(1);
    meshROS.pose.orientation.y = q(2);
    meshROS.pose.orientation.z = q(3);
    meshROS.scale.x = 2.0;
    meshROS.scale.y = 2.0;
    meshROS.scale.z = 2.0;
    meshROS.color.a = color_a_;
    meshROS.color.r = color_r_;
    meshROS.color.g = color_g_;
    meshROS.color.b = color_b_;
    meshROS.mesh_resource = mesh_resource_;
    mesh_pub_->publish(meshROS);
  }

  // Parameters
  std::string mesh_resource_;
  double color_r_, color_g_, color_b_, color_a_, cov_scale_, scale_;
  bool cross_config_, tf45_, cov_pos_, cov_vel_, cov_color_, origin_;
  bool is_origin_set_;
  std::string frame_id_;
  int drone_id_;

  // State
  colvec poseOrigin_;
  nav_msgs::msg::Path path_ros_;
  rclcpp::Time prevt_{0, 0, RCL_ROS_TIME};
  bool prevt_initialized_;
  rclcpp::Time pt_{0, 0, RCL_ROS_TIME};
  bool pt_initialized_;
  colvec ppose_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cov_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cov_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr height_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_cmd_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomVisualization>());
  rclcpp::shutdown();
  return 0;
}
