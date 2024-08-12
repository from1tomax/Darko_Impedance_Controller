#ifndef BASE_IMPEDANCE_CONTROLLER_H
#define BASE_IMPEDANCE_CONTROLLER_H

#include "model.h"
#include "eigen3/Eigen/Eigenvalues"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"

struct ConfigurationSpace
{
  Eigen::Matrix<double, NUMJ_VEH, 1> q_v;//under the world coordinate system
  Eigen::Matrix<double, NUMJ_ARM, 1> q_a;
  Eigen::Matrix<double, NUMJ_VEH, 1> dq_v;// under the base coordinate system
  Eigen::Matrix<double, NUMJ_ARM, 1> dq_a;
  Eigen::Matrix<double, NUMJ_VEH, 1> ddq_v;
  Eigen::Matrix<double, NUMJ_ARM, 1> ddq_a;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CartesianSpace
{
  Eigen::Matrix<double, 3, 1> p;
  Eigen::Matrix3d R;
  Eigen::Matrix<double, 6, 1> dX;
  Eigen::Matrix<double, 6, 1> ddX;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class BaseImpedanceControl
{
public:
  /*---- ATTRIBUTES ----*/

  /*Body chosen*/
  int body;

  /*Sampling time*/
  double ts;

  /*Communication flag*/
  bool received_des_traj = false;

  /*Configurtion space quantities*/
  Eigen::MatrixXd M; // Mass Matrix
  Eigen::MatrixXd C; // Coriolis Matrix
  Eigen::VectorXd G; // Gravity Vector

  Eigen::MatrixXd J;  // Body Jacobian Matrix
  Eigen::MatrixXd dJ; // Derivative of configuration Jacobian
  Eigen::Matrix4d K;  // Forward Kinematics

  ConfigurationSpace robot_config;
  ConfigurationSpace old_robot_config;

  ConfigurationSpace command;
  ConfigurationSpace old_command;

  /*Cartesian space quantities*/
  Eigen::Matrix<double, 6, 6> Lambda; // Mass Matrix
  Eigen::Matrix<double, 6, 6> mu;     // Coriolis Matrix
  Eigen::MatrixXd J_x;                // Cartesian Jacobian (quaternion representation)
  Eigen::MatrixXd J_x_old;            // Cartesian Jacobian (quaternion representation) at previous sampling time
  Eigen::MatrixXd dJ_x;               // Derivative of Cartesian Jacobian (quaternion representation)
  Eigen::MatrixXd J_x_i;              // Dinamically consistent pseudo-inverse (quaternion representation)
  Eigen::Matrix<double, 6, 1> v_t;    // Cartesian desired velocity (quaternion representation modified problem)
  Eigen::Matrix<double, 6, 1> dv_t;   // Cartesian desired acceleration (quaternion representation modified problem)
  Eigen::Matrix<double, 6, 1> X_err;  // Pose error
  Eigen::Matrix<double, 6, 1> dX_err; // Velocity error

  CartesianSpace robot_cartesian;
  CartesianSpace robot_cartesian_initial;
  CartesianSpace robot_desired;

  /*Null space coefficients*/
  Eigen::VectorXd Kp;
  Eigen::VectorXd Kv;

  /*Impedance coefficients*/
  Eigen::Matrix<double, 6, 6> Kc; // Stiffness
  Eigen::Matrix<double, 6, 6> Dc; // Damping
  double zeta;                    // Damping factor, must be in interval: [0,1]

  Eigen::VectorXd _tau;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*---- METHODS ----*/
  BaseImpedanceControl(std::string choose_body, double ts);

  void whole_body_control_torque(CartesianSpace trajectory);

  Eigen::VectorXd null_space_torque(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a, Eigen::VectorXd q_a_des);

  void impedance_control_torque();

  void compute_kinematics(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a);

  void compute_configuration_quantities(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a);

  void compute_cartesian_quantities(Eigen::Matrix3d R_W_D, Eigen::VectorXd dX_des, Eigen::VectorXd ddX_des);

  void compute_damping();

  void compute_error(Eigen::Matrix3d R_W_D, Eigen::VectorXd p_des, Eigen::VectorXd dX_des, Eigen::Matrix3d R, Eigen::VectorXd p, Eigen::VectorXd dX);

  void admittance_interface();

  void read_desired_trajectory(CartesianSpace trajectory);

  /*---- TOPIC RELATED ----*/

  void base_config_callback(const nav_msgs::Odometry &odom_msg);

  void arm_config_callback(const boost::shared_ptr<const sensor_msgs::JointState> p_joint_state_msg);

  void initialize_kinematics(ros::NodeHandle &n);
};

/* Utility functions */
Eigen::Matrix3d calc_skewsymm(Eigen::Vector3d v);
Eigen::MatrixXd augment_matrix(Eigen::Matrix3d M);
Eigen::MatrixXd time_derivative(Eigen::MatrixXd A, Eigen::MatrixXd A_old, double ts);
Eigen::VectorXd time_integration(Eigen::VectorXd v_int0, Eigen::VectorXd v, Eigen::VectorXd v_old, double ts);

#endif