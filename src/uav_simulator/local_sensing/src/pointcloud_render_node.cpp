#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

class PclRenderNode : public rclcpp::Node
{
public:
  PclRenderNode() : Node("pcl_render")
  {
    if (!this->has_parameter("sensing_horizon")) {
      this->declare_parameter<double>("sensing_horizon", 5.0);
    }
    if (!this->has_parameter("sensing_rate")) {
      this->declare_parameter<double>("sensing_rate", 10.0);
    }
    if (!this->has_parameter("estimation_rate")) {
      this->declare_parameter<double>("estimation_rate", 10.0);
    }
    if (!this->has_parameter("map.x_size")) {
      this->declare_parameter<double>("map.x_size", 50.0);
    }
    if (!this->has_parameter("map.y_size")) {
      this->declare_parameter<double>("map.y_size", 50.0);
    }
    if (!this->has_parameter("map.z_size")) {
      this->declare_parameter<double>("map.z_size", 5.0);
    }

    this->get_parameter("sensing_horizon", sensing_horizon_);
    this->get_parameter("sensing_rate", sensing_rate_);
    this->get_parameter("estimation_rate", estimation_rate_);
    this->get_parameter("map.x_size", x_size_);
    this->get_parameter("map.y_size", y_size_);
    this->get_parameter("map.z_size", z_size_);

    // subscribe point cloud
    global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/global_map", 1,
        std::bind(&PclRenderNode::rcvGlobalPointCloudCallBack, this, std::placeholders::_1));
    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/local_map", 1,
        std::bind(&PclRenderNode::rcvLocalPointCloudCallBack, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odometry", 50,
        std::bind(&PclRenderNode::rcvOdometryCallbck, this, std::placeholders::_1));

    // publisher depth image and color image
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/pcl_render_node/cloud", 10);

    double sensing_duration = 1.0 / sensing_rate_ * 2.5;

    local_sensing_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(sensing_duration),
        std::bind(&PclRenderNode::renderSensedPoints, this));

    inv_resolution_ = 1.0 / resolution_;

    gl_xl_ = -x_size_ / 2.0;
    gl_yl_ = -y_size_ / 2.0;
    gl_zl_ = 0.0;

    GLX_SIZE_ = (int)(x_size_ * inv_resolution_);
    GLY_SIZE_ = (int)(y_size_ * inv_resolution_);
    GLZ_SIZE_ = (int)(z_size_ * inv_resolution_);
  }

private:
  void rcvOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    has_odom_ = true;
    odom_ = *odom;
  }

  void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
  {
    if (has_global_map_) return;

    RCLCPP_WARN(this->get_logger(), "Global Pointcloud received..");

    pcl::PointCloud<pcl::PointXYZ> cloud_input;
    pcl::fromROSMsg(*pointcloud_map, cloud_input);

    voxel_sampler_.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_sampler_.setInputCloud(cloud_input.makeShared());
    voxel_sampler_.filter(cloud_all_map_);

    kdtree_local_map_.setInputCloud(cloud_all_map_.makeShared());

    has_global_map_ = true;
  }

  void rcvLocalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr /*pointcloud_map*/)
  {
    // do nothing, fix later
  }

  void renderSensedPoints()
  {
    if (!has_global_map_ || !has_odom_) return;

    Eigen::Quaterniond q;
    q.x() = odom_.pose.pose.orientation.x;
    q.y() = odom_.pose.pose.orientation.y;
    q.z() = odom_.pose.pose.orientation.z;
    q.w() = odom_.pose.pose.orientation.w;

    Eigen::Matrix3d rot;
    rot = q;
    Eigen::Vector3d yaw_vec = rot.col(0);

    local_map_.points.clear();
    pcl::PointXYZ searchPoint(odom_.pose.pose.position.x,
                              odom_.pose.pose.position.y,
                              odom_.pose.pose.position.z);
    point_idx_radius_search_.clear();
    point_radius_squared_distance_.clear();

    pcl::PointXYZ pt;
    if (kdtree_local_map_.radiusSearch(searchPoint, sensing_horizon_,
                                        point_idx_radius_search_,
                                        point_radius_squared_distance_) > 0) {
      for (size_t i = 0; i < point_idx_radius_search_.size(); ++i) {
        pt = cloud_all_map_.points[point_idx_radius_search_[i]];

        if ((fabs(pt.z - odom_.pose.pose.position.z) / sensing_horizon_) >
            tan(M_PI / 6.0))
          continue;

        Vector3d pt_vec(pt.x - odom_.pose.pose.position.x,
                        pt.y - odom_.pose.pose.position.y,
                        pt.z - odom_.pose.pose.position.z);

        if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue;

        local_map_.points.push_back(pt);
      }
    } else {
      return;
    }

    local_map_.width = local_map_.points.size();
    local_map_.height = 1;
    local_map_.is_dense = true;

    pcl::toROSMsg(local_map_, local_map_pcd_);
    local_map_pcd_.header.frame_id = "world";

    pub_cloud_->publish(local_map_pcd_);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_, local_map_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr local_sensing_timer_;

  // State
  bool has_global_map_{false};
  bool has_local_map_{false};
  bool has_odom_{false};

  nav_msgs::msg::Odometry odom_;

  double sensing_horizon_{5.0}, sensing_rate_{10.0}, estimation_rate_{10.0};
  double x_size_{50.0}, y_size_{50.0}, z_size_{5.0};
  double gl_xl_{0.0}, gl_yl_{0.0}, gl_zl_{0.0};
  double resolution_{0.1}, inv_resolution_{10.0};
  int GLX_SIZE_{0}, GLY_SIZE_{0}, GLZ_SIZE_{0};

  pcl::PointCloud<pcl::PointXYZ> cloud_all_map_, local_map_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler_;
  sensor_msgs::msg::PointCloud2 local_map_pcd_;

  pcl::search::KdTree<pcl::PointXYZ> kdtree_local_map_;
  vector<int> point_idx_radius_search_;
  vector<float> point_radius_squared_distance_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclRenderNode>());
  rclcpp::shutdown();
  return 0;
}
