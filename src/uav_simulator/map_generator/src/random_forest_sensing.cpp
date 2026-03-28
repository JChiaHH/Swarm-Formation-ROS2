#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include <random>

using namespace std;

class RandomForestSensing : public rclcpp::Node
{
public:
  RandomForestSensing() : Node("random_map_sensing")
  {
    _local_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/local_cloud", 1);
    _all_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", 1);
    click_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcl_render_node/local_map", 1);

    if (!this->has_parameter("map.x_size")) {
      this->declare_parameter<double>("map.x_size", 50.0);
    }
    if (!this->has_parameter("map.y_size")) {
      this->declare_parameter<double>("map.y_size", 50.0);
    }
    if (!this->has_parameter("map.z_size")) {
      this->declare_parameter<double>("map.z_size", 5.0);
    }
    if (!this->has_parameter("map.obs_num")) {
      this->declare_parameter<int>("map.obs_num", 30);
    }
    if (!this->has_parameter("map.resolution")) {
      this->declare_parameter<double>("map.resolution", 0.1);
    }
    if (!this->has_parameter("map.circle_num")) {
      this->declare_parameter<int>("map.circle_num", 30);
    }
    if (!this->has_parameter("ObstacleShape.lower_rad")) {
      this->declare_parameter<double>("ObstacleShape.lower_rad", 0.3);
    }
    if (!this->has_parameter("ObstacleShape.upper_rad")) {
      this->declare_parameter<double>("ObstacleShape.upper_rad", 0.8);
    }
    if (!this->has_parameter("ObstacleShape.lower_hei")) {
      this->declare_parameter<double>("ObstacleShape.lower_hei", 3.0);
    }
    if (!this->has_parameter("ObstacleShape.upper_hei")) {
      this->declare_parameter<double>("ObstacleShape.upper_hei", 7.0);
    }
    if (!this->has_parameter("ObstacleShape.radius_l")) {
      this->declare_parameter<double>("ObstacleShape.radius_l", 7.0);
    }
    if (!this->has_parameter("ObstacleShape.radius_h")) {
      this->declare_parameter<double>("ObstacleShape.radius_h", 7.0);
    }
    if (!this->has_parameter("ObstacleShape.z_l")) {
      this->declare_parameter<double>("ObstacleShape.z_l", 7.0);
    }
    if (!this->has_parameter("ObstacleShape.z_h")) {
      this->declare_parameter<double>("ObstacleShape.z_h", 7.0);
    }
    if (!this->has_parameter("ObstacleShape.theta")) {
      this->declare_parameter<double>("ObstacleShape.theta", 7.0);
    }
    if (!this->has_parameter("fix_obs_type")) {
      this->declare_parameter<int>("fix_obs_type", 0);
    }
    if (!this->has_parameter("pub_rate")) {
      this->declare_parameter<double>("pub_rate", 10.0);
    }
    if (!this->has_parameter("min_distance")) {
      this->declare_parameter<double>("min_distance", 1.0);
    }

    this->get_parameter("map.x_size", _x_size);
    this->get_parameter("map.y_size", _y_size);
    this->get_parameter("map.z_size", _z_size);
    this->get_parameter("map.obs_num", _obs_num);
    this->get_parameter("map.resolution", _resolution);
    this->get_parameter("map.circle_num", circle_num_);
    this->get_parameter("ObstacleShape.lower_rad", _w_l);
    this->get_parameter("ObstacleShape.upper_rad", _w_h);
    this->get_parameter("ObstacleShape.lower_hei", _h_l);
    this->get_parameter("ObstacleShape.upper_hei", _h_h);
    this->get_parameter("ObstacleShape.radius_l", radius_l_);
    this->get_parameter("ObstacleShape.radius_h", radius_h_);
    this->get_parameter("ObstacleShape.z_l", z_l_);
    this->get_parameter("ObstacleShape.z_h", z_h_);
    this->get_parameter("ObstacleShape.theta", theta_);
    this->get_parameter("fix_obs_type", fix_obs_type_);
    this->get_parameter("pub_rate", _pub_rate);
    this->get_parameter("min_distance", _min_dist);

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;
    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    _obs_num = min(_obs_num, (int)_x_size * 10);
    _z_limit = _z_size;

    if (!this->has_parameter("ObstacleShape.seed")) {
      this->declare_parameter<int>("ObstacleShape.seed", -1);
    }
    int seed_param = -1;
    this->get_parameter("ObstacleShape.seed", seed_param);
    unsigned int seed = (seed_param >= 0) ? static_cast<unsigned int>(seed_param) : rd();
    cout << "seed = " << seed << " ~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~~!~" << endl;
    eng.seed(seed);

    RandomMapGenerateCylinder();

    // Timer-based publish loop
    pub_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / _pub_rate),
        std::bind(&RandomForestSensing::pubPoints, this));
  }

private:
  void GenerateWall(double x_l, double x_h,
                    double y_l, double y_h,
                    double z_l, double z_h,
                    pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    int x_num, y_num, z_num;
    x_num = ceil((x_h - x_l) / _resolution);
    y_num = ceil((y_h - y_l) / _resolution);
    z_num = ceil((z_h - z_l) / _resolution);
    pcl::PointXYZ pt;
    for (int i = 0; i < x_num; i++)
      for (int j = 0; j < y_num; j++)
        for (int k = 0; k < z_num; k++) {
          pt.x = x_l + i * _resolution;
          pt.y = y_l + j * _resolution;
          pt.z = z_l + k * _resolution;
          cloud.push_back(pt);
        }
  }

  void RandomMapGenerateCylinder()
  {
    pcl::PointXYZ pt_random;

    vector<Eigen::Vector2d> obs_position;

    rand_x = uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = uniform_real_distribution<double>(_h_l, _h_h);
    rand_inf = uniform_real_distribution<double>(0.5, 1.5);

    rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num && rclcpp::ok(); i++) {
      double x, y, w, inf;
      x = rand_x(eng);
      y = rand_y(eng);
      w = rand_w(eng);
      inf = rand_inf(eng);

      bool flag_continue = false;
      for (auto p : obs_position)
        if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist) {
          i--;
          flag_continue = true;
          break;
        }
      if (flag_continue) continue;

      obs_position.push_back(Eigen::Vector2d(x, y));

      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil((w * inf) / _resolution);
      double radius = (w * inf) / 2;

      for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
          double h = rand_h(eng);
          int heiNum = ceil(h / _resolution);
          for (int t = -10; t < heiNum; t++) {
            double temp_x = x + (r + 0.5) * _resolution + 1e-2;
            double temp_y = y + (s + 0.5) * _resolution + 1e-2;
            double temp_z = (t + 0.5) * _resolution + 1e-2;
            if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius) {
              pt_random.x = temp_x;
              pt_random.y = temp_y;
              pt_random.z = temp_z;
              cloudMap.points.push_back(pt_random);
            }
          }
        }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
      double x, y, z;
      x = rand_x(eng);
      y = rand_y(eng);
      z = rand_z_(eng);

      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;
      z = floor(z / _resolution) * _resolution + _resolution / 2.0;

      Eigen::Vector3d translate(x, y, z);

      double theta = rand_theta_(eng);
      Eigen::Matrix3d rotate;
      rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

      double radius1 = rand_radius_(eng);
      double radius2 = rand_radius2_(eng);

      Eigen::Vector3d cpt;
      for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
        cpt(0) = 0.0;
        cpt(1) = radius1 * cos(angle);
        cpt(2) = radius2 * sin(angle);

        Eigen::Vector3d cpt_if;
        for (int ifx = -0; ifx <= 0; ++ifx)
          for (int ify = -0; ify <= 0; ++ify)
            for (int ifz = -0; ifz <= 0; ++ifz) {
              cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                             ifz * _resolution);
              cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
              pt_random.x = cpt_if(0);
              pt_random.y = cpt_if(1);
              pt_random.z = cpt_if(2);
              cloudMap.push_back(pt_random);
            }
      }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    RCLCPP_WARN(this->get_logger(), "Finished generate random map ");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
  }

  void clickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double w = rand_w(eng);
    double h;
    pcl::PointXYZ pt_random;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -1; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          clicked_cloud_.points.push_back(pt_random);
          cloudMap.points.push_back(pt_random);
        }
      }
    clicked_cloud_.width = clicked_cloud_.points.size();
    clicked_cloud_.height = 1;
    clicked_cloud_.is_dense = true;

    sensor_msgs::msg::PointCloud2 localMap_pcd;
    pcl::toROSMsg(clicked_cloud_, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    click_map_pub_->publish(localMap_pcd);

    cloudMap.width = cloudMap.points.size();
  }

  void pubPoints()
  {
    if (!_map_ok) return;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    _all_map_pub->publish(globalMap_pcd);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _local_map_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr click_map_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Random
  random_device rd;
  default_random_engine eng;
  uniform_real_distribution<double> rand_x;
  uniform_real_distribution<double> rand_y;
  uniform_real_distribution<double> rand_w;
  uniform_real_distribution<double> rand_h;
  uniform_real_distribution<double> rand_inf;
  uniform_real_distribution<double> rand_radius_;
  uniform_real_distribution<double> rand_radius2_;
  uniform_real_distribution<double> rand_theta_;
  uniform_real_distribution<double> rand_z_;

  // PCL
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;

  sensor_msgs::msg::PointCloud2 globalMap_pcd;
  pcl::PointCloud<pcl::PointXYZ> cloudMap;
  pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

  // Parameters
  int _obs_num{30};
  double _x_size{50.0}, _y_size{50.0}, _z_size{5.0};
  double _x_l{0.0}, _x_h{0.0}, _y_l{0.0}, _y_h{0.0}, _w_l{0.3}, _w_h{0.8}, _h_l{3.0}, _h_h{7.0};
  double _z_limit{5.0}, _sensing_range{0.0}, _resolution{0.1}, _pub_rate{10.0};
  double _min_dist{1.0};
  int fix_obs_type_{0};

  int circle_num_{30};
  double radius_l_{7.0}, radius_h_{7.0}, z_l_{7.0}, z_h_{7.0};
  double theta_{7.0};

  bool _map_ok{false};
  bool _has_odom{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomForestSensing>());
  rclcpp::shutdown();
  return 0;
}
