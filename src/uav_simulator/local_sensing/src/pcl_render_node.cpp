#include <iostream>
#include <fstream>
#include <vector>
// include ros dep.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

// include pcl dep
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// include opencv and eigen
#include <Eigen/Eigen>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "depth_render.cuh"
#include "quadrotor_msgs/msg/position_command.hpp"
using namespace cv;
using namespace std;
using namespace Eigen;

class PclRenderCudaNode : public rclcpp::Node
{
public:
  PclRenderCudaNode() : Node("pcl_render")
  {
    this->declare_parameter<int>("cam_width", 640);
    this->declare_parameter<int>("cam_height", 480);
    this->declare_parameter<double>("cam_fx", 387.229248046875);
    this->declare_parameter<double>("cam_fy", 387.229248046875);
    this->declare_parameter<double>("cam_cx", 321.04638671875);
    this->declare_parameter<double>("cam_cy", 243.44969177246094);
    this->declare_parameter<double>("sensing_horizon", 5.0);
    this->declare_parameter<double>("sensing_rate", 10.0);
    this->declare_parameter<double>("estimation_rate", 10.0);
    this->declare_parameter<double>("map.x_size", 50.0);
    this->declare_parameter<double>("map.y_size", 50.0);
    this->declare_parameter<double>("map.z_size", 5.0);

    this->get_parameter("cam_width", width_);
    this->get_parameter("cam_height", height_);
    this->get_parameter("cam_fx", fx_);
    this->get_parameter("cam_fy", fy_);
    this->get_parameter("cam_cx", cx_);
    this->get_parameter("cam_cy", cy_);
    this->get_parameter("sensing_horizon", sensing_horizon_);
    this->get_parameter("sensing_rate", sensing_rate_);
    this->get_parameter("estimation_rate", estimation_rate_);
    this->get_parameter("map.x_size", x_size_);
    this->get_parameter("map.y_size", y_size_);
    this->get_parameter("map.z_size", z_size_);

    depthrender_.set_para(fx_, fy_, cx_, cy_, width_, height_);

    cam02body_ << 0.0, 0.0, 1.0, 0.0,
                 -1.0, 0.0, 0.0, 0.0,
                  0.0, -1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;

    cam2world_ = Matrix4d::Identity();

    // subscribe point cloud
    global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/global_map", 1,
        std::bind(&PclRenderCudaNode::rcvGlobalPointCloudCallBack, this, std::placeholders::_1));
    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/local_map", 1,
        std::bind(&PclRenderCudaNode::rcvLocalPointCloudCallBack, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odometry", 50,
        std::bind(&PclRenderCudaNode::rcvOdometryCallbck, this, std::placeholders::_1));

    // publisher depth image and color image
    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("~/depth", 1000);
    pub_color_ = this->create_publisher<sensor_msgs::msg::Image>("~/colordepth", 1000);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", 1000);
    pub_pcl_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/rendered_pcl", 1);

    double sensing_duration = 1.0 / sensing_rate_;
    double estimate_duration = 1.0 / estimation_rate_;

    local_sensing_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(sensing_duration),
        std::bind(&PclRenderCudaNode::renderSensedPoints, this));
    estimation_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(estimate_duration),
        std::bind(&PclRenderCudaNode::pubCameraPose, this));

    inv_resolution_ = 1.0 / resolution_;

    gl_xl_ = -x_size_ / 2.0;
    gl_yl_ = -y_size_ / 2.0;
    gl_zl_ = 0.0;

    GLX_SIZE_ = (int)(x_size_ * inv_resolution_);
    GLY_SIZE_ = (int)(y_size_ * inv_resolution_);
    GLZ_SIZE_ = (int)(z_size_ * inv_resolution_);
  }

private:
  inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index)
  {
    Eigen::Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * resolution_ + gl_xl_;
    pt(1) = ((double)index(1) + 0.5) * resolution_ + gl_yl_;
    pt(2) = ((double)index(2) + 0.5) * resolution_ + gl_zl_;
    return pt;
  }

  inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt)
  {
    Eigen::Vector3i idx;
    idx(0) = std::min(std::max(int((pt(0) - gl_xl_) * inv_resolution_), 0), GLX_SIZE_ - 1);
    idx(1) = std::min(std::max(int((pt(1) - gl_yl_) * inv_resolution_), 0), GLY_SIZE_ - 1);
    idx(2) = std::min(std::max(int((pt(2) - gl_zl_) * inv_resolution_), 0), GLZ_SIZE_ - 1);
    return idx;
  }

  void rcvOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    has_odom_ = true;
    odom_ = *odom;
    Matrix4d Pose_receive = Matrix4d::Identity();

    Eigen::Vector3d request_position;
    Eigen::Quaterniond request_pose;
    request_position.x() = odom->pose.pose.position.x;
    request_position.y() = odom->pose.pose.position.y;
    request_position.z() = odom->pose.pose.position.z;
    request_pose.x() = odom->pose.pose.orientation.x;
    request_pose.y() = odom->pose.pose.orientation.y;
    request_pose.z() = odom->pose.pose.orientation.z;
    request_pose.w() = odom->pose.pose.orientation.w;
    Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
    Pose_receive(0, 3) = request_position(0);
    Pose_receive(1, 3) = request_position(1);
    Pose_receive(2, 3) = request_position(2);

    Matrix4d body_pose = Pose_receive;
    cam2world_ = body_pose * cam02body_;
    cam2world_quat_ = cam2world_.block<3, 3>(0, 0);

    last_odom_stamp_ = odom->header.stamp;

    last_pose_world_(0) = odom->pose.pose.position.x;
    last_pose_world_(1) = odom->pose.pose.position.y;
    last_pose_world_(2) = odom->pose.pose.position.z;
  }

  void pubCameraPose()
  {
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = odom_.header;
    camera_pose.header.frame_id = "/map";
    camera_pose.pose.position.x = cam2world_(0, 3);
    camera_pose.pose.position.y = cam2world_(1, 3);
    camera_pose.pose.position.z = cam2world_(2, 3);
    camera_pose.pose.orientation.w = cam2world_quat_.w();
    camera_pose.pose.orientation.x = cam2world_quat_.x();
    camera_pose.pose.orientation.y = cam2world_quat_.y();
    camera_pose.pose.orientation.z = cam2world_quat_.z();
    pub_pose_->publish(camera_pose);
  }

  void renderSensedPoints()
  {
    if (!has_global_map_ && !has_local_map_) return;
    if (!has_odom_) return;
    render_currentpose();
    render_pcl_world();
  }

  void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
  {
    if (has_global_map_) return;

    RCLCPP_WARN(this->get_logger(), "Global Pointcloud received..");
    pcl::PointCloud<pcl::PointXYZ> cloudIn;
    pcl::PointXYZ pt_in;
    pcl::fromROSMsg(*pointcloud_map, cloudIn);
    for (int i = 0; i < int(cloudIn.points.size()); i++) {
      pt_in = cloudIn.points[i];
      cloud_data_.push_back(pt_in.x);
      cloud_data_.push_back(pt_in.y);
      cloud_data_.push_back(pt_in.z);
    }
    printf("global map has points: %d.\n", (int)cloud_data_.size() / 3);
    depthrender_.set_data(cloud_data_);
    depth_hostptr_ = (int*)malloc(width_ * height_ * sizeof(int));

    has_global_map_ = true;
  }

  void rcvLocalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
  {
    pcl::PointCloud<pcl::PointXYZ> cloudIn;
    pcl::PointXYZ pt_in;
    pcl::fromROSMsg(*pointcloud_map, cloudIn);

    if (cloudIn.points.size() == 0) return;
    for (int i = 0; i < int(cloudIn.points.size()); i++) {
      pt_in = cloudIn.points[i];
      Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
      cloud_data_.push_back(pose_pt(0));
      cloud_data_.push_back(pose_pt(1));
      cloud_data_.push_back(pose_pt(2));
    }
    depthrender_.set_data(cloud_data_);
    depth_hostptr_ = (int*)malloc(width_ * height_ * sizeof(int));

    has_local_map_ = true;
  }

  void render_pcl_world()
  {
    pcl::PointCloud<pcl::PointXYZ> localMap;
    pcl::PointXYZ pt_in;

    Eigen::Vector4d pose_in_camera;
    Eigen::Vector4d pose_in_world;
    Eigen::Vector3d pose_pt;

    for (int u = 0; u < width_; u++)
      for (int v = 0; v < height_; v++) {
        float depth = depth_mat_.at<float>(v, u);

        if (depth == 0.0) continue;

        pose_in_camera(0) = (u - cx_) * depth / fx_;
        pose_in_camera(1) = (v - cy_) * depth / fy_;
        pose_in_camera(2) = depth;
        pose_in_camera(3) = 1.0;

        pose_in_world = cam2world_ * pose_in_camera;

        if ((pose_in_world.segment(0, 3) - last_pose_world_).norm() > sensing_horizon_)
          continue;

        pose_pt = pose_in_world.head(3);
        pt_in.x = pose_pt(0);
        pt_in.y = pose_pt(1);
        pt_in.z = pose_pt(2);

        localMap.points.push_back(pt_in);
      }

    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    pcl::toROSMsg(localMap, local_map_pcl_);
    local_map_pcl_.header.frame_id = "/map";
    local_map_pcl_.header.stamp = last_odom_stamp_;

    pub_pcl_world_->publish(local_map_pcl_);
  }

  void render_currentpose()
  {
    Matrix4d cam_pose = cam2world_.inverse();

    double pose[4 * 4];

    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++)
        pose[j + 4 * i] = cam_pose(i, j);

    depthrender_.render_pose(pose, depth_hostptr_);

    depth_mat_ = cv::Mat::zeros(height_, width_, CV_32FC1);
    double max = 1.0f;
    for (int i = 0; i < height_; i++)
      for (int j = 0; j < width_; j++) {
        float depth = (float)depth_hostptr_[i * width_ + j] / 1000.0f;
        depth = depth < 500.0f ? depth : 0;
        max = depth > max ? depth : max;
        depth_mat_.at<float>(i, j) = depth;
      }

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = last_odom_stamp_;
    out_msg.header.frame_id = "camera";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image = depth_mat_.clone();
    pub_depth_->publish(*out_msg.toImageMsg());

    cv::Mat adjMap;
    double min = 0.5;
    depth_mat_.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
    cv::Mat falseColorsMap;
    cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
    cv_bridge::CvImage cv_image_colored;
    cv_image_colored.header.frame_id = "depthmap";
    cv_image_colored.header.stamp = last_odom_stamp_;
    cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image_colored.image = falseColorsMap;
    pub_color_->publish(*cv_image_colored.toImageMsg());
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_world_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_, local_map_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr local_sensing_timer_, estimation_timer_;

  // State
  bool has_global_map_{false};
  bool has_local_map_{false};
  bool has_odom_{false};

  int width_{640}, height_{480};
  double fx_{387.0}, fy_{387.0}, cx_{321.0}, cy_{243.0};

  DepthRender depthrender_;
  int* depth_hostptr_{nullptr};
  cv::Mat depth_mat_;

  Matrix4d cam02body_;
  Matrix4d cam2world_;
  Eigen::Quaterniond cam2world_quat_;
  nav_msgs::msg::Odometry odom_;

  double sensing_horizon_{5.0}, sensing_rate_{10.0}, estimation_rate_{10.0};
  double x_size_{50.0}, y_size_{50.0}, z_size_{5.0};
  double gl_xl_{0.0}, gl_yl_{0.0}, gl_zl_{0.0};
  double resolution_{0.1}, inv_resolution_{10.0};
  int GLX_SIZE_{0}, GLY_SIZE_{0}, GLZ_SIZE_{0};

  Eigen::Vector3d last_pose_world_{Eigen::Vector3d::Zero()};
  builtin_interfaces::msg::Time last_odom_stamp_;

  sensor_msgs::msg::PointCloud2 local_map_pcl_;
  sensor_msgs::msg::PointCloud2 local_depth_pcl_;

  vector<float> cloud_data_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclRenderCudaNode>());
  rclcpp::shutdown();
  return 0;
}
