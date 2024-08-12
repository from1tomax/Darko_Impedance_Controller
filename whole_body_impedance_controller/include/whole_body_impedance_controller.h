#ifndef IMPEDANCECONTROL_H
#define IMPEDANCECONTROL_H

#include "model.h"
#include "eigen3/Eigen/Eigenvalues"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"

struct ConfigurationSpace
{
  Eigen::Matrix<double, NUMJ_VEH, 1> q_v;// vehicle position under the world coordinate system;
  Eigen::Matrix<double, NUMJ_ARM, 1> q_a;// arm joint angle under each joint coordinate system;
  Eigen::Matrix<double, NUMJ_VEH, 1> dq_v;// vehicle velocity under the "base coordinate system"-->that is important, because it is different from q_v;
  Eigen::Matrix<double, NUMJ_ARM, 1> dq_a;// arm joint velocity under each joint coordinate system;
  Eigen::Matrix<double, NUMJ_VEH, 1> ddq_v;// vehicle acceleration under the base coordinate system
  Eigen::Matrix<double, NUMJ_ARM, 1> ddq_a;// arm joint acceleration under each joint coordinate system;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CartesianSpace
{
  Eigen::Matrix<double, 3, 1> p; //position in cartesian space under the world coordinate system
  Eigen::Matrix3d R;// R is the rotation matrix 3*3 from T_W_EE， describes how EE rotates under the world coordinate system
  Eigen::Matrix<double, 6, 1> dX;//dX is the velocity of EE in cartesian space under the world coordinate system
  Eigen::Matrix<double, 6, 1> ddX;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ImpedanceControl{
public:

  /*---- ATTRIBUTES ----*/
  

  /*Sampling time*/
  double ts;
  Eigen::Matrix4d T_V_A;
  
  /*Configurtion space quantities*/
  Eigen::MatrixXd M;                 //Mass Matrix
  Eigen::MatrixXd C;                 //Coriolis Matrix
  Eigen::VectorXd G;                 //Gravity Vector

  Eigen::MatrixXd J;                 //Body Jacobian Matrix
  Eigen::MatrixXd dJ;              //Derivative of configuration Jacobian
  Eigen::Matrix4d T_W_EE;                 //Forward Kynematics

  ConfigurationSpace robot_config;
  ConfigurationSpace old_robot_config;

  ConfigurationSpace command;
  ConfigurationSpace old_command;

  /*Cartesian space quantities*/
  Eigen::Matrix<double, 6,6> Lambda; //Mass Matrix
  Eigen::Matrix<double, 6,6> mu;     //Coriolis Matrix
  Eigen::MatrixXd J_x;               //Cartesian Jacobian (quaternion representation)
  Eigen::MatrixXd dJ_x;             //Derivative of Cartesian Jacobian (quaternion representation)
  Eigen::MatrixXd J_x_i;             //Dinamically consistent pseudo-inverse (quaternion representation)
  Eigen::Matrix<double, 6,1> v_t;    //Cartesian desired velocity (quaternion representation modified problem)
  Eigen::Matrix<double, 6,1> dv_t;  //Cartesian desired acceleration (quaternion representation modified problem)
  Eigen::Matrix<double, 6,1> X_err;  //Pose error
  Eigen::Matrix<double, 6,1> dX_err; //Velocity error

  CartesianSpace robot_cartesian;
  CartesianSpace robot_desired;

  /*Null space coefficients*/
  Eigen::VectorXd Kp;
  Eigen::VectorXd Kv;
  
  /*Impedance coefficients*/
  Eigen::Matrix<double, 6,6> Kc; //Stiffness
  Eigen::Matrix<double, 6,6> Dc;       //Damping
  double zeta;                         //Damping factor, must be in interval: [0,1]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*---- METHODS ----*/
  ImpedanceControl(double time_step);

  Eigen::VectorXd whole_body_control_torque(Eigen::VectorXd q_a_des);

  Eigen::VectorXd null_space_torque(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a, Eigen::VectorXd q_a_des);

  Eigen::VectorXd impedance_control_torque();

  void compute_kinematics(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a);

  void compute_configuration_quantities(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a);

  void compute_cartesian_quantities(Eigen::Matrix3d R_W_D, Eigen::VectorXd dX_des, Eigen::VectorXd ddX_des);
  
  void compute_damping();

  void compute_error(Eigen::Matrix3d R_W_D, Eigen::VectorXd p_des, Eigen::VectorXd dX_des, Eigen::Matrix3d R, Eigen::VectorXd p, Eigen::VectorXd dX);
  
  void admittance_interface(Eigen::VectorXd tau, double dt);

  void read_desired_trajectory();
  
  /*---- TOPIC RELATED ----*/
  
  void base_config_callback(const nav_msgs::Odometry& odom_msg);

  void arm_config_callback(const boost::shared_ptr<const sensor_msgs::JointState> p_joint_state_msg);

  void initialize_kinematics(ros::NodeHandle &n);

};

/* Utility functions */
Eigen::Matrix3d calc_skewsymm(Eigen::Vector3d v);
Eigen::MatrixXd augment_matrix(Eigen::Matrix3d M);
Eigen::MatrixXd time_derivative(Eigen::MatrixXd A, Eigen::MatrixXd A_old, double ts);
Eigen::VectorXd time_integration(Eigen::VectorXd v_int0, Eigen::VectorXd v, Eigen::VectorXd v_old, double ts);

#endif