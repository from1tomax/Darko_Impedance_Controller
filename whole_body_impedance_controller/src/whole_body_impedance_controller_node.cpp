#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include "model.h"
#include "whole_body_impedance_controller.h"
#include <vector>


#include <sstream>

std::vector<double> eigen_matrix_to_vector(Eigen::MatrixXd mat);
std::vector<double> eigen_vector_to_std_vector(const Eigen::VectorXd& vec);
void signal_callback_handler(int signum);



int main(int argc, char **argv)
{
  
  Eigen::VectorXd q_a_des(NUMJ_ARM); 
  q_a_des << 0.008802080165774662, 0.0017394970902619988, 0.029336759329098072, -0.06978132904768408, -0.017805467852986645, -0.0006683489704197854, 0.0004275592394078487;

  // Instantiate and initialize controller
  ImpedanceControl controller(0.1);

  // Initialize ROS node and topics
  ros::init(argc, argv, "whole_body_impedance_controller_node");
  ros::NodeHandle n;

  // Subscribe to get the current trajectory attained by the robot
  ros::Subscriber base_odometry_sub = n.subscribe("/darko/robotnik_base_control/odom", 1, &ImpedanceControl::base_config_callback, &controller);
  ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1, &ImpedanceControl::arm_config_callback, &controller);


  std_msgs::Float64MultiArray arm_command;
  geometry_msgs::Twist base_command;
  ros::Publisher panda_joint_pos_pub = n.advertise<std_msgs::Float64MultiArray>("/gazebo_panda/effort_joint_torque_controller/command", 0);
  ros::Publisher base_velocity_pub = n.advertise<geometry_msgs::Twist>("/darko/robotnik_base_control/cmd_vel", 0);

 

  // Rate at which the node is to be run
  ros::Rate loop_rate(10);
  
  // Read initial robot pose 
  controller.initialize_kinematics(n);
  
  // Initialize motion generator

  // controller.robot_desired.p << 2.4286, 2.6592, 1.6101;
  controller.robot_desired.p << 2.4, 2.7, 1.4;
  controller.robot_desired.R<<  0.9955, 0.0666, -0.0678,
                                0.0666, -0.9978, -0.0033,
                                -0.0679, -0.0012, -0.9977;
  controller.robot_desired.dX = controller.robot_cartesian.dX;
  controller.robot_desired.ddX << 0, 0, 0, 0, 0, 0;
  
  ros::Time last_time = ros::Time::now();

  while (ros::ok()) {
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;//calculate the real time interval between two loops;

  controller.robot_desired.p(0) += 0.1*dt;

    ROS_INFO_STREAM("time_step: " <<std::fixed<<std::setprecision(4)<< dt);
    ROS_INFO_STREAM("robot_config.q_v: " <<std::fixed<<std::setprecision(4)<< controller.robot_config.q_v.transpose());

    ROS_INFO_STREAM("robot_desired.p: " <<std::fixed<<std::setprecision(4)<< controller.robot_desired.p.transpose());
    ROS_INFO_STREAM("robot_desired.R: " <<std::fixed<<std::setprecision(4)<< controller.robot_desired.R);
    ROS_INFO_STREAM("robot_desired.dX: " <<std::fixed<<std::setprecision(4)<< controller.robot_desired.dX.transpose());
    ROS_INFO_STREAM("robot_desired.ddX: " <<std::fixed<<std::setprecision(4)<< controller.robot_desired.ddX.transpose());

    ROS_INFO_STREAM("robot_cartesian.p: " <<std::fixed<<std::setprecision(4)<< controller.robot_cartesian.p.transpose());
    ROS_INFO_STREAM("robot_cartesian.R: " <<std::fixed<<std::setprecision(4)<< controller.robot_cartesian.R);
    ROS_INFO_STREAM("robot_cartesian.dX: " <<std::fixed<<std::setprecision(4)<< controller.robot_cartesian.dX.transpose());
    ROS_INFO_STREAM("robot_cartesian.ddX: " <<std::fixed<<std::setprecision(4)<< controller.robot_cartesian.ddX.transpose());

    ROS_INFO_STREAM("robot_cartesian.X_err: " <<std::fixed<<std::setprecision(4)<< controller.X_err.transpose());
    ROS_INFO_STREAM("robot_cartesian.dX_err: " <<std::fixed<<std::setprecision(4)<< controller.dX_err.transpose());

    ROS_INFO_STREAM("J_x: " <<std::fixed<<std::setprecision(4)<< controller.J_x);
    

    Eigen::VectorXd tau = controller.whole_body_control_torque(q_a_des);

    controller.admittance_interface(tau,dt);
    
    base_command.linear.x = 0.1*controller.command.dq_v.coeff(0,0); 
    base_command.linear.y = 0.1*controller.command.dq_v.coeff(1,0);
    base_command.angular.z = 0.1*controller.command.dq_v.coeff(2,0);

    arm_command.data = eigen_vector_to_std_vector(tau.tail(7));

    base_velocity_pub.publish(base_command);
    panda_joint_pos_pub.publish(arm_command);

   ROS_INFO_STREAM("tau_arm: " <<std::fixed<<std::setprecision(4)<< tau.tail(7).transpose());
   ROS_INFO_STREAM("tau_base: " <<std::fixed<<std::setprecision(4)<< tau.head(3).transpose());
   ROS_INFO_STREAM("command.dq_v: " <<std::fixed<<std::setprecision(4)<< controller.command.dq_v.transpose());
   ROS_INFO_STREAM("G: " <<std::fixed<<std::setprecision(4)<< controller.G.transpose());
   std::cout << "------------------------------------------------------------------------------------------" << std::endl;

    signal(SIGINT, signal_callback_handler); //ensure to stop the robot when ctrl+C is pressed                                                 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

/* Function definitions */

std::vector<double> eigen_matrix_to_vector(Eigen::MatrixXd mat) {
    std::vector<double> v(mat.size());
    Eigen::VectorXd::Map(&v[0], mat.size()) = mat;
    return v;
}

std::vector<double> eigen_vector_to_std_vector(const Eigen::VectorXd& vec) {
    return std::vector<double>(vec.data(), vec.data() + vec.size());
}

/// @brief Handles the Ctrl+C signal from terminal. Stops the robot base.
/// @param signum 
void signal_callback_handler(int signum) {
  ROS_INFO("Whole Body Control is going to be stopped...");
  std_msgs::Float64MultiArray arm_command;
  geometry_msgs::Twist base_command;

  ros::NodeHandle n;
  ros::Publisher panda_joint_pos_pub = n.advertise<std_msgs::Float64MultiArray>("/gazebo_panda/effort_joint_position_controller/command", 0);
  ros::Publisher base_velocity_pub = n.advertise<geometry_msgs::Twist>("/darko/robotnik_base_control/cmd_vel", 0);

  base_command.linear.x = 0; 
  base_command.linear.y = 0;
  base_command.angular.z = 0;

  std::vector<double> q_a_initial = {0.008802080165774662, 0.0017394970902619988, 0.029336759329098072, -0.06978132904768408, -0.017805467852986645, -0.0006683489704197854, 0.0004275592394078487};
  arm_command.data = q_a_initial;

  panda_joint_pos_pub.publish(arm_command);
  base_velocity_pub.publish(base_command);
  // Terminate program
  ros::shutdown();
}

