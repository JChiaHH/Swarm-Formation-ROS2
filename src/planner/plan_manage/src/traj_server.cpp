#include <nav_msgs/msg/odometry.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

using namespace std::chrono_literals;

class TrajServer : public rclcpp::Node
{
public:
  TrajServer() : Node("traj_server")
  {
    pos_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 50);

    poly_traj_sub_ = this->create_subscription<traj_utils::msg::PolyTraj>(
        "planning/trajectory", 10,
        std::bind(&TrajServer::polyTrajCallback, this, std::placeholders::_1));

    reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "planning/finish", 10,
        std::bind(&TrajServer::finishCallback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "planning/start", 10,
        std::bind(&TrajServer::startCallback, this, std::placeholders::_1));

    cmd_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.01),
        std::bind(&TrajServer::cmdCallback, this));

    /* control parameter */
    cmd_.kx[0] = pos_gain_[0];
    cmd_.kx[1] = pos_gain_[1];
    cmd_.kx[2] = pos_gain_[2];

    cmd_.kv[0] = vel_gain_[0];
    cmd_.kv[1] = vel_gain_[1];
    cmd_.kv[2] = vel_gain_[2];

    if (!this->has_parameter("traj_server/time_forward")) {
      this->declare_parameter<double>("traj_server/time_forward", -1.0);
    }
    this->get_parameter("traj_server/time_forward", time_forward_);
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    // get drone name for result file
    std::string name_drone = this->get_name();
    std::vector<std::string> v{explode(name_drone, '_')};
    if (v.size() > 1)
    {
      result_file_.open(result_dir_ + v[1] + "_vaj.txt", std::ios::out);
    }

    RCLCPP_WARN(this->get_logger(), "[Traj server]: ready.");
  }

private:
  static const std::vector<std::string> explode(const std::string &s, const char &c)
  {
    std::string buff{""};
    std::vector<std::string> v;

    for (auto n : s)
    {
      if (n != c)
        buff += n;
      else if (n == c && buff != "")
      {
        v.push_back(buff);
        buff = "";
      }
    }
    if (buff != "")
      v.push_back(buff);

    return v;
  }

  void polyTrajCallback(const traj_utils::msg::PolyTraj::SharedPtr msg)
  {
    if (msg->order != 5)
    {
      RCLCPP_ERROR(this->get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
      RCLCPP_ERROR(this->get_logger(), "[traj_server] WRONG trajectory parameters, ");
      return;
    }

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    traj_.reset(new poly_traj::Trajectory(dura, cMats));

    start_time_ = msg->start_time;
    traj_duration_ = traj_->getTotalDuration();
    traj_id_ = msg->traj_id;

    receive_traj_ = true;
  }

  void finishCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data == true)
    {
      try {
        RCLCPP_WARN(this->get_logger(), "total_time, cnt, acc2_inter, jerk2_inter = %lf \t %d \t %lf \t %lf",
                    (this->now() - global_start_time_).seconds(), cnt_, acc2_inter_, jerk2_inter_);
      } catch (const std::runtime_error&) {
        RCLCPP_WARN(this->get_logger(), "cnt, acc2_inter, jerk2_inter = %d \t %lf \t %lf",
                    cnt_, acc2_inter_, jerk2_inter_);
      }
      auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::stringstream ss;
      ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
      std::string str_time = ss.str();
      result_file_ << str_time << "\n";
      double max_vel2 = 0;
      for (size_t i = 0; i < time_vec_.size(); i++)
      {
        double tmp_vel2 = (vel_vec_[i](0)) * (vel_vec_[i](0)) + (vel_vec_[i](1)) * (vel_vec_[i](1)) + (vel_vec_[i](2)) * (vel_vec_[i](2));
        max_vel2 = (tmp_vel2 > max_vel2) ? tmp_vel2 : max_vel2;
      }
      result_file_ << "max_vel = " << sqrt(max_vel2) << "\n";
      result_file_ << "\n";
    }
  }

  void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data == true)
    {
      RCLCPP_WARN(this->get_logger(), "START!!!!");
      global_start_time_ = this->now();
    }
  }

  std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
  {
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                              ? traj_->getPos(t_cur + time_forward_) - pos
                              : traj_->getPos(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1
                          ? atan2(dir(1), dir(0))
                          : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
    if (yaw_temp - last_yaw_ > PI)
    {
      if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
      {
        yaw = last_yaw_ - max_yaw_change;
        if (yaw < -PI)
          yaw += 2 * PI;

        yawdot = -YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if (yaw - last_yaw_ > PI)
          yawdot = -YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
      }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
      if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
      {
        yaw = last_yaw_ + max_yaw_change;
        if (yaw > PI)
          yaw -= 2 * PI;

        yawdot = YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if (yaw - last_yaw_ < -PI)
          yawdot = YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
      }
    }
    else
    {
      if (yaw_temp - last_yaw_ < -max_yaw_change)
      {
        yaw = last_yaw_ - max_yaw_change;
        if (yaw < -PI)
          yaw += 2 * PI;

        yawdot = -YAW_DOT_MAX_PER_SEC;
      }
      else if (yaw_temp - last_yaw_ > max_yaw_change)
      {
        yaw = last_yaw_ + max_yaw_change;
        if (yaw > PI)
          yaw -= 2 * PI;

        yawdot = YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if (yaw - last_yaw_ > PI)
          yawdot = -YAW_DOT_MAX_PER_SEC;
        else if (yaw - last_yaw_ < -PI)
          yawdot = YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
      }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
      yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
  }

  void cmdCallback()
  {
    /* no publishing before receive traj_ */
    if (!receive_traj_)
      return;

    rclcpp::Time time_now = this->now();
    double t_cur = (time_now - start_time_).seconds();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), pos_f;
    std::pair<double, double> yaw_yawdot(0, 0);

    if (!time_last_initialized_)
    {
      time_last_ = this->now();
      time_last_initialized_ = true;
    }

    if (flag_ == false)
    {
      flag_ = true;
    }
    else
    {
      cnt_++;
      acc2_inter_ += last_acc_.norm() * last_acc_.norm() * (time_now - time_last_).seconds();
      jerk2_inter_ += last_jerk_.norm() * last_jerk_.norm() * (time_now - time_last_).seconds();
    }
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
      pos = traj_->getPos(t_cur);
      vel = traj_->getVel(t_cur);
      acc = traj_->getAcc(t_cur);
      jerk = traj_->getJer(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last_);
      /*** calculate yaw ***/

      double tf = std::min(traj_duration_, t_cur + 2.0);
      pos_f = traj_->getPos(tf);
    }
    else if (t_cur >= traj_duration_)
    {
      /* hover when finish traj_ */
      pos = traj_->getPos(traj_duration_);
      vel.setZero();
      acc.setZero();

      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0;

      pos_f = pos;
    }
    else
    {
      // std::cout << "[Traj server]: invalid time." << std::endl;
    }
    time_last_ = time_now;
    time_vec_.push_back((this->now() - global_start_time_).seconds());
    pos_vec_.push_back(pos);
    vel_vec_.push_back(vel);
    acc_vec_.push_back(acc);
    jerk_vec_.push_back(jerk);
    last_vel_ = vel;
    last_acc_ = acc;
    last_jerk_ = jerk;

    cmd_.header.stamp = time_now;
    cmd_.header.frame_id = "world";
    cmd_.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd_.trajectory_id = traj_id_;

    cmd_.position.x = pos(0);
    cmd_.position.y = pos(1);
    cmd_.position.z = pos(2);

    cmd_.velocity.x = vel(0);
    cmd_.velocity.y = vel(1);
    cmd_.velocity.z = vel(2);

    cmd_.acceleration.x = acc(0);
    cmd_.acceleration.y = acc(1);
    cmd_.acceleration.z = acc(2);

    cmd_.yaw = yaw_yawdot.first;
    cmd_.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd_.yaw;

    pos_cmd_pub_->publish(cmd_);
  }

  // Publishers / Subscribers / Timers
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Subscription<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // State
  quadrotor_msgs::msg::PositionCommand cmd_;
  double pos_gain_[3] = {0, 0, 0};
  double vel_gain_[3] = {0, 0, 0};

  bool receive_traj_ = false;
  std::shared_ptr<poly_traj::Trajectory> traj_;
  double traj_duration_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  int traj_id_;

  // yaw control
  double last_yaw_ = 0.0, last_yaw_dot_ = 0.0;
  double time_forward_;

  Eigen::Vector3d last_vel_{Eigen::Vector3d::Zero()}, last_acc_{Eigen::Vector3d::Zero()}, last_jerk_{Eigen::Vector3d::Zero()};
  bool flag_ = false;
  double jerk2_inter_ = 0, acc2_inter_ = 0;
  int cnt_ = 0;
  rclcpp::Time global_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time time_last_{0, 0, RCL_ROS_TIME};
  bool time_last_initialized_ = false;
  int drone_id_;
  std::string result_dir_ = "/home/jeremychia/Swarm-Formation-ROS2/results/";
  std::fstream result_file_;
  std::vector<Eigen::Vector3d> pos_vec_, vel_vec_, acc_vec_, jerk_vec_;
  std::vector<double> time_vec_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TrajServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
