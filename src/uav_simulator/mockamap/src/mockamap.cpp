#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "world";
  RCLCPP_INFO(in.node->get_logger(), "finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

class MockamapNode : public rclcpp::Node
{
public:
  MockamapNode() : Node("mockamap")
  {
    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);

    if (!this->has_parameter("seed")) {
      this->declare_parameter<int>("seed", 4546);
    }
    if (!this->has_parameter("update_freq")) {
      this->declare_parameter<double>("update_freq", 1.0);
    }
    if (!this->has_parameter("resolution")) {
      this->declare_parameter<double>("resolution", 0.38);
    }
    if (!this->has_parameter("x_length")) {
      this->declare_parameter<int>("x_length", 100);
    }
    if (!this->has_parameter("y_length")) {
      this->declare_parameter<int>("y_length", 100);
    }
    if (!this->has_parameter("z_length")) {
      this->declare_parameter<int>("z_length", 10);
    }
    if (!this->has_parameter("type")) {
      this->declare_parameter<int>("type", 1);
    }

    int seed;
    double update_freq;
    double scale;
    int sizeX, sizeY, sizeZ;
    int type;

    this->get_parameter("seed", seed);
    this->get_parameter("update_freq", update_freq);
    this->get_parameter("resolution", scale);
    this->get_parameter("x_length", sizeX);
    this->get_parameter("y_length", sizeY);
    this->get_parameter("z_length", sizeZ);
    this->get_parameter("type", type);

    scale = 1 / scale;
    sizeX = sizeX * scale;
    sizeY = sizeY * scale;
    sizeZ = sizeZ * scale;

    mocka::Maps::BasicInfo info;
    info.node       = this;
    info.sizeX      = sizeX;
    info.sizeY      = sizeY;
    info.sizeZ      = sizeZ;
    info.seed       = seed;
    info.scale      = scale;
    info.output     = &output_;
    info.cloud      = &cloud_;

    mocka::Maps map;
    map.setInfo(info);
    map.generate(type);

    // publish loop via timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / update_freq),
        [this]() {
          pcl_pub_->publish(output_);
        });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  sensor_msgs::msg::PointCloud2 output_;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockamapNode>());
  rclcpp::shutdown();
  return 0;
}
