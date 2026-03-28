#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include <traj_utils/msg/assignment.hpp>
#include "poly_traj_utils.hpp"
#include "munkres_algorithm.hpp"
#include <swarm_graph/swarm_graph.hpp>
#include <fstream>

namespace ego_planner
{

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  private:
    rclcpp::Node::SharedPtr node_;
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstrainPoints cps_;
    SwarmGraph::Ptr swarm_graph_;

    int drone_id_;
    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_;     // optimization variables
    int piece_num_;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm

    string result_fn_;
    fstream result_file_;

    double collision_check_time_end_ = 0.0;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    enum FORMATION_TYPE
    {
      NONE_FORMATION        = 0,
      REGULAR_HEXAGON       = 1,
      STAR_FORMATION        = 2,
      ARROW_FORMATION       = 3,
      SUTD_FORMATION        = 4
    };

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_swarm_;                       // swarm weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double wei_formation_;                   // swarm formation simllarity

    double obs_clearance_;                   // safe distance between uav and obstacles
    double swarm_clearance_;                 // safe distance between uav and uav
    double max_vel_, max_acc_;               // dynamic limits

    int    formation_type_;
    int    formation_size_;
    bool   use_formation_ = true;
    bool   is_other_assigning_ = false;

    double t_now_;

  public:

    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    void setParam(rclcpp::Node::SharedPtr node);
    void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    double getCollisionCheckTimeEnd() { return collision_check_time_end_; }

    /* main planning API */
    bool OptimizeTrajectory_lbfgs(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points, const bool use_formation);

    void astarWithMinTraj( const Eigen::MatrixXd &iniState,
                           const Eigen::MatrixXd &finState,
                           std::vector<Eigen::Vector3d> &simple_path,
                           Eigen::MatrixXd &ctl_points,
                           poly_traj::MinJerkOpt &frontendMJ);



  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                              const Eigen::Vector3d &p,
                              Eigen::Vector3d &gradp,
                              double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool swarmGraphGradCostP(const int i_dp,
                             const double t,
                             const Eigen::Vector3d &p,
                             const Eigen::Vector3d &v,
                             Eigen::Vector3d &gradp,
                             double &gradt,
                             double &grad_prev_t,
                             double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void showFormationInformation(bool is_show, Eigen::Vector3d pos);

    bool checkCollision(void);

    bool getFormationPos(std::vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos);

    void setDesiredFormation(int type){
      std::vector<Eigen::Vector3d> swarm_des;
      switch (type)
      {
        case FORMATION_TYPE::NONE_FORMATION :
        {
          use_formation_  = false;
          formation_size_ = 0;
          break;
        }

        case FORMATION_TYPE::REGULAR_HEXAGON :
        {
          // set the desired formation
          Eigen::Vector3d v0(0,0,0);
          Eigen::Vector3d v1(1.7321,-1,0);
          Eigen::Vector3d v2(0,-2,0);
          Eigen::Vector3d v3(-1.7321,-1,0);
          Eigen::Vector3d v4(-1.7321,1,0);
          Eigen::Vector3d v5(0,2,0);
          Eigen::Vector3d v6(1.7321,1,0);

          swarm_des.push_back(v0);
          swarm_des.push_back(v1);
          swarm_des.push_back(v2);
          swarm_des.push_back(v3);
          swarm_des.push_back(v4);
          swarm_des.push_back(v5);
          swarm_des.push_back(v6);

          formation_size_ = swarm_des.size();
          swarm_graph_->setDesiredForm(swarm_des);
          break;
        }

        case FORMATION_TYPE::STAR_FORMATION :
        {
          // 10-point star: 5 outer tips (r=2.0) + 5 inner valleys (r=0.8)
          // Starting at 90 deg (top), alternating every 36 deg
          // Inner radius increased from 0.8 to 1.2 for better spacing (min 1.4m vs 0.94m)
          swarm_des.push_back(Eigen::Vector3d( 0.0000,  2.0000, 0));  // outer tip top
          swarm_des.push_back(Eigen::Vector3d(-0.7060,  0.9710, 0));  // inner (r=1.2)
          swarm_des.push_back(Eigen::Vector3d(-1.9021,  0.6180, 0));  // outer tip left
          swarm_des.push_back(Eigen::Vector3d(-1.1410, -0.3710, 0));  // inner (r=1.2)
          swarm_des.push_back(Eigen::Vector3d(-1.1756, -1.6180, 0));  // outer tip lower-left
          swarm_des.push_back(Eigen::Vector3d( 0.0000, -1.2000, 0));  // inner (r=1.2)
          swarm_des.push_back(Eigen::Vector3d( 1.1756, -1.6180, 0));  // outer tip lower-right
          swarm_des.push_back(Eigen::Vector3d( 1.1410, -0.3710, 0));  // inner (r=1.2)
          swarm_des.push_back(Eigen::Vector3d( 1.9021,  0.6180, 0));  // outer tip right
          swarm_des.push_back(Eigen::Vector3d( 0.7060,  0.9710, 0));  // inner (r=1.2)

          formation_size_ = swarm_des.size();
          swarm_graph_->setDesiredForm(swarm_des);
          break;
        }

        case FORMATION_TYPE::ARROW_FORMATION :
        {
          // 10-drone arrow/V-formation: wider spacing, shorter tip, offset tail
          swarm_des.push_back(Eigen::Vector3d( 1.5,   0.0, 0));  // tip
          swarm_des.push_back(Eigen::Vector3d( 0.75,  0.6, 0));  // left wing 1
          swarm_des.push_back(Eigen::Vector3d( 0.0,   1.2, 0));  // left wing 2
          swarm_des.push_back(Eigen::Vector3d(-0.75,  1.8, 0));  // left wing 3
          swarm_des.push_back(Eigen::Vector3d(-1.5,   2.4, 0));  // left wing 4
          swarm_des.push_back(Eigen::Vector3d( 0.75, -0.6, 0));  // right wing 1
          swarm_des.push_back(Eigen::Vector3d( 0.0,  -1.2, 0));  // right wing 2
          swarm_des.push_back(Eigen::Vector3d(-0.75, -1.8, 0));  // right wing 3
          swarm_des.push_back(Eigen::Vector3d(-1.5,  -2.4, 0));  // right wing 4
          swarm_des.push_back(Eigen::Vector3d(-1.5,   0.0, 0));  // tail center

          formation_size_ = swarm_des.size();
          swarm_graph_->setDesiredForm(swarm_des);
          break;
        }

        case FORMATION_TYPE::SUTD_FORMATION :
        {
          // 30-drone SUTD text (compact font): S(7) + U(9) + T(6) + D(8)
          // S
          swarm_des.push_back(Eigen::Vector3d(-6.2414,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-5.2414,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-6.2414,  0.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-6.2414, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d(-5.2414, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d(-5.2414, -1.5862, 0));
          // U
          swarm_des.push_back(Eigen::Vector3d(-3.2414,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-1.2414,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-3.2414,  0.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-1.2414,  0.4138, 0));
          swarm_des.push_back(Eigen::Vector3d(-3.2414, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d(-1.2414, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d(-3.2414, -1.5862, 0));
          swarm_des.push_back(Eigen::Vector3d(-2.2414, -1.5862, 0));
          swarm_des.push_back(Eigen::Vector3d(-1.2414, -1.5862, 0));
          // T
          swarm_des.push_back(Eigen::Vector3d( 0.7586,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 1.7586,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 2.7586,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 1.7586,  0.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 1.7586, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d( 1.7586, -1.5862, 0));
          // D
          swarm_des.push_back(Eigen::Vector3d( 4.7586,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 5.7586,  1.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 4.7586,  0.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 6.7586,  0.4138, 0));
          swarm_des.push_back(Eigen::Vector3d( 4.7586, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d( 6.7586, -0.5862, 0));
          swarm_des.push_back(Eigen::Vector3d( 4.7586, -1.5862, 0));
          swarm_des.push_back(Eigen::Vector3d( 5.7586, -1.5862, 0));
          // Extra S bottom-left (drone 29)
          swarm_des.push_back(Eigen::Vector3d(-6.2414, -1.5862, 0));

          formation_size_ = swarm_des.size();
          swarm_graph_->setDesiredForm(swarm_des);
          break;
        }

        default:
          break;
      }
    }

  public:

    void changeFormation(int type) { setDesiredFormation(type); }

    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace ego_planner
#endif