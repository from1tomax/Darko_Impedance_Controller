#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include "arm_impedance_controller.h"
#include <iostream>
#include <iomanip>

std::vector<double> eigen_matrix_to_vector(Eigen::MatrixXd mat);
std::vector<double> eigen_vector_to_std_vector(const Eigen::VectorXd& vec);
void signal_callback_handler(int signum);

int main(int argc, char **argv)
{

  Eigen::VectorXd q_a_des(NUMJ_ARM); 
  q_a_des << 0., 0., 0., 0., 0., 0., 0.;
  // Instantiate and initialize Armcontroller
  ArmImpedanceControl Armcontroller(0.1);

  // Initialize ROS node and topics
  ros::init(argc, argv, "whole_body_controller_node");
  ros::NodeHandle n;

  //define start time and duration
  ros::Time start_time = ros::Time::now();
  ros::Duration period;

  // Subscribe to get the current trajectory attained by the robot
  ros::Subscriber base_odometry_sub = n.subscribe("/darko/robotnik_base_control/odom", 1, &ArmImpedanceControl::base_config_callback, &Armcontroller);
  ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1, &ArmImpedanceControl::arm_config_callback, &Armcontroller);

  //msg for control
  std_msgs::Float64MultiArray arm_command;
  geometry_msgs::Twist base_command;
  //msg for plotting
  std_msgs::Float64MultiArray current_EE_pose;
  std_msgs::Float64MultiArray desired_EE_pose;
  std_msgs::Float64MultiArray pose_error;
  
  ros::Publisher panda_joint_pos_pub = n.advertise<std_msgs::Float64MultiArray>("/gazebo_panda/effort_joint_torque_controller/command", 0);
  ros::Publisher base_velocity_pub = n.advertise<geometry_msgs::Twist>("/darko/robotnik_base_control/cmd_vel", 0);
  ros::Publisher current_EE_pose_pub = n.advertise<std_msgs::Float64MultiArray>("/darko/current_EE_pose", 0);
  ros::Publisher desired_EE_pose_pub = n.advertise<std_msgs::Float64MultiArray>("/darko/desired_EE_pose", 0);
  ros::Publisher pose_error_pub = n.advertise<std_msgs::Float64MultiArray>("/darko/pose_error", 0);

  
  // Rate at which the node is to be run
  ros::Rate loop_rate(10);
  
  // Read initial robot pose 
  Armcontroller.initialize_kinematics(n);
  // q_a_des = Armcontroller.robot_config.q_a;
  Armcontroller.robot_desired.R = Armcontroller.robot_cartesian.R;
  // Armcontroller.robot_desired.p = Armcontroller.robot_cartesian.p;
  Armcontroller.robot_desired.p << 2.4, 2.7, 1.4;
  Armcontroller.robot_desired.dX = Armcontroller.robot_cartesian.dX;
  Armcontroller.robot_desired.ddX = Armcontroller.robot_cartesian.ddX;

  //EE initial position
  // x=2.44 y=2.66 z=1.62
  double x = 2.44;
  double y = 2.66;
  double z = 1.5;
  double total_seconds = 0;
  bool is_near = false;

  while (ros::ok()) {
      
      
      //motion with sin
      // total_seconds +=0.1;
      // if(!is_near){
      //   x = 2.7;
      //   if(fabs(Armcontroller.robot_cartesian.p[0]-2.6) < 0.03 || fabs(Armcontroller.robot_cartesian.p[2]-1.4) < 0.03){
      //   is_near = true;
      //   }
      // }else{
      //   x = 2.7 + 0.2*sin(total_seconds);
      // }
      // ROS_INFO_STREAM("sin(total_seconds): " << sin(total_seconds));
      // ROS_INFO_STREAM("x: " << x);
        // ROS_INFO_STREAM("sin(total_seconds): " << sin(total_seconds));

      // if(!is_near){
      //   z = 1.5;
      //   if(fabs(Armcontroller.robot_cartesian.p[2]-1.5) < 0.01){
      //   is_near = true;
      //   }
      // }else{
      //   total_seconds +=0.1;
      //   x = 2.4+0.1*total_seconds;
      //   if(x>2.8){
      //     x = 2.8;
      //   }
      // }
      // total_seconds +=0.1;
      // x = 2.4+0.1*total_seconds;
      // if (x>2.8)
      // {
      //   x = 2.8;
      // }
      x = 2.6;
      y = 2.66;
      z = 1.4;
      Armcontroller.robot_desired.p << x, y, z;

      



      Eigen::VectorXd tau_nullspace = Armcontroller.null_space_torque(Armcontroller.robot_config.q_a, Armcontroller.robot_config.dq_a, q_a_des);
      Eigen::VectorXd tau_impedance = Armcontroller.impedance_control_torque();
      Eigen::VectorXd tau = Armcontroller.whole_body_control_torque(q_a_des);// tau = tau_nullspace + tau_impedance;
      //update and publish msgs 
      arm_command.data = eigen_vector_to_std_vector(tau);

      base_command.linear.x = 0;
      base_command.linear.y = 0;
      base_command.angular.z = 0;

      current_EE_pose.data = eigen_matrix_to_vector(Armcontroller.robot_cartesian.p);
      desired_EE_pose.data = {x,y,z};
      pose_error.data = eigen_matrix_to_vector(Armcontroller.X_err);


      current_EE_pose_pub.publish(current_EE_pose);
      desired_EE_pose_pub.publish(desired_EE_pose);
      pose_error_pub.publish(pose_error);

      base_velocity_pub.publish(base_command);
      panda_joint_pos_pub.publish(arm_command);

      //print info
      ROS_INFO_STREAM("config.qa: " << Armcontroller.robot_config.q_a.transpose());

      ROS_INFO_STREAM("robot_cartesian.R: " << Armcontroller.robot_cartesian.R.transpose());
      ROS_INFO_STREAM("robot_cartesian.p: " << Armcontroller.robot_cartesian.p.transpose());
      ROS_INFO_STREAM("period " << period.toSec());
      // ROS_INFO_STREAM("x: " << x << " y: " << y << " z: " << z);
      ROS_INFO_STREAM("robot_desired.p: " << Armcontroller.robot_desired.p.transpose());
      ROS_INFO_STREAM("robot_cartesian.dX: " << Armcontroller.robot_cartesian.dX.transpose());
      ROS_INFO_STREAM("gravity_compesnsation: " << Armcontroller.G.transpose());
      ROS_INFO_STREAM("X_err: "<< Armcontroller.X_err.transpose());
      ROS_INFO_STREAM("dX_err: "<< Armcontroller.dX_err.transpose());
      ROS_INFO_STREAM("velocity: " << Armcontroller.v_t.transpose());
      ROS_INFO_STREAM("acceleration: " << Armcontroller.dv_t.transpose());
      // ROS_INFO_STREAM("Tau: " << tau.transpose());
      ROS_INFO_STREAM("Tau_nullspace: " << tau_nullspace.transpose());
      ROS_INFO_STREAM("Tau_impedance: " << tau_impedance.transpose());
      ROS_INFO_STREAM("Tau_command: " << arm_command.data[0] << " " << arm_command.data[1] << " " << arm_command.data[2] << " " 
      << arm_command.data[3] << " " << arm_command.data[4] << " " << arm_command.data[5] << " " << arm_command.data[6]);
      ROS_INFO_STREAM("-----------------------------------------");

 
      // if(Armcontroller.robot_config.dq_a.norm() > 150) return 0;




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

  std::vector<double> tau = {0., 0., 0., 0., 0., 0., 0.};
  arm_command.data = tau;

  panda_joint_pos_pub.publish(arm_command);
  base_velocity_pub.publish(base_command);
  // Terminate program
  ros::shutdown();
}


