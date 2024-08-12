#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include "base_impedance_controller.h"
#include <iostream>
#include <iomanip>

void signal_callback_handler(int signum);
std::vector<double> eigen_matrix_to_vector(Eigen::MatrixXd mat);

int main(int argc, char **argv)
{

    // Eigen::VectorXd q_a_des(NUMJ_ARM);
    // q_a_des << 0., -0.785, 0., -2.356, 0., 1.57, 0.785;
    BaseImpedanceControl base_controller("VEH", 0.1);

    ros::init(argc, argv, "base_impedance_controller");
    ros::NodeHandle n;
    // set start_time and duration
    ros::Time start_time = ros::Time::now();
    ros::Duration period;

    ros::Subscriber base_odometry_sub = n.subscribe("/darko/robotnik_base_control/odom", 1, &BaseImpedanceControl::base_config_callback, &base_controller);
    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1, &BaseImpedanceControl::arm_config_callback, &base_controller);

    base_controller.initialize_kinematics(n);

    ros::Publisher base_vel_pub = n.advertise<geometry_msgs::Twist>("/darko/robotnik_base_control/cmd_vel", 10);

    ros::Publisher debug_Xerr_dXerr = n.advertise<std_msgs::Float64MultiArray>("/pose_error", 10);
    ros::Publisher debug_vt_dvt = n.advertise<std_msgs::Float64MultiArray>("/debug_vt_dvt", 10);
    ros::Publisher debug_tau = n.advertise<std_msgs::Float64MultiArray>("/debug_tau", 10);
    ros::Publisher debug_vel = n.advertise<std_msgs::Float64MultiArray>("/debug_vel", 10);

    geometry_msgs::Twist base_command; // Create a Twist message object

    std_msgs::Float64MultiArray pose_error;
    std_msgs::Float64MultiArray debug_vt_dvt_msg;
    std_msgs::Float64MultiArray debug_tau_msg;
    std_msgs::Float64MultiArray debug_vel_msg;

    CartesianSpace trajectory;
    trajectory.p << 2.07427, 2.5, 0.345;
    trajectory.R << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    trajectory.dX << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    trajectory.ddX << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    ros::Rate loop_rate(10); // Set the frequency to 10 Hz

    while (ros::ok())
    {
        // use to debug
        Eigen::VectorXd debug_Xerr_dXerr_vec(6);
        Eigen::VectorXd debug_vt_dvt_vec(6);
        Eigen::VectorXd debug_tau_vec(3);
        Eigen::VectorXd debug_vel_vec(3);

        debug_Xerr_dXerr_vec << base_controller.X_err(0),
            base_controller.X_err(1),
            base_controller.X_err(5),
            base_controller.dX_err(0),
            base_controller.dX_err(1),
            base_controller.dX_err(5);

        debug_vt_dvt_vec << base_controller.v_t(0),
            base_controller.v_t(1),
            base_controller.v_t(5),
            base_controller.dv_t(0),
            base_controller.dv_t(1),
            base_controller.dv_t(5);

        debug_tau_vec << base_controller._tau(0),
            base_controller._tau(1),
            base_controller._tau(2);

        debug_vel_vec << base_controller.command.dq_v(0),
            base_controller.command.dq_v(1),
            base_controller.command.dq_v(2);

        pose_error.data = eigen_matrix_to_vector(debug_Xerr_dXerr_vec);
        debug_vt_dvt_msg.data = eigen_matrix_to_vector(debug_vt_dvt_vec);
        debug_tau_msg.data = eigen_matrix_to_vector(debug_tau_vec);
        debug_vel_msg.data = eigen_matrix_to_vector(debug_vel_vec);

        debug_Xerr_dXerr.publish(pose_error);
        debug_vt_dvt.publish(debug_vt_dvt_msg);
        debug_tau.publish(debug_tau_msg);
        debug_vel.publish(debug_vel_msg);
        // end of debug

        // if (trajectory.p(0)<=5.0) {
        //     trajectory.p(0) = trajectory.p(0) + 0.05;
        //     trajectory.dX(0) = -0.05;
        // } else {
        //     trajectory.p(0)=5.0;
        //     trajectory.dX(0) = 0.0;
        // }
        trajectory.p(0) = 5.0;

        // double angle = 0.5 * sin(0.1 * period.toSec());
        // trajectory.R << -cos(angle), sin(angle), 0,
        //     -sin(angle), -cos(angle), 0,
        //     0, 0, 1;


        ROS_INFO_STREAM("robot_config.q_v: " << std::fixed << std::setprecision(4) << base_controller.robot_config.q_v.transpose());
        ROS_INFO_STREAM("robot_config.dq_v: " << std::fixed << std::setprecision(4) << base_controller.robot_config.dq_v.transpose());

        ROS_INFO_STREAM("robot_cartesian.P: " << std::fixed << std::setprecision(4) << base_controller.robot_cartesian.p.transpose());
        ROS_INFO_STREAM("robot_cartesian.R: " << std::fixed << std::setprecision(4) << base_controller.robot_cartesian.R);
        ROS_INFO_STREAM("robot_cartesian.dX: " << std::fixed << std::setprecision(4) << base_controller.robot_cartesian.dX.transpose());
        ROS_INFO_STREAM("robot_cartesian.ddX: " << std::fixed << std::setprecision(4) << base_controller.robot_cartesian.ddX.transpose());

        ROS_INFO_STREAM("robot_desired.P: " << std::fixed << std::setprecision(4) << base_controller.robot_desired.p.transpose());
        ROS_INFO_STREAM("robot_desired.R: " << std::fixed << std::setprecision(4) << base_controller.robot_desired.R);
        ROS_INFO_STREAM("robot_desired.dX: " << std::fixed << std::setprecision(4) << base_controller.robot_desired.dX.transpose());
        ROS_INFO_STREAM("robot_desired.ddX: " << std::fixed << std::setprecision(4) << base_controller.robot_desired.ddX.transpose());

        ROS_INFO_STREAM("X_err: " << std::fixed << std::setprecision(4) << base_controller.X_err.transpose());
        ROS_INFO_STREAM("dX_err: " << std::fixed << std::setprecision(4) << base_controller.dX_err.transpose());

        ROS_INFO_STREAM("desired_vel(v_t): " << std::fixed << std::setprecision(4) << base_controller.v_t.transpose());
        ROS_INFO_STREAM("desired_acc(dv_t): " << std::fixed << std::setprecision(4) << base_controller.dv_t.transpose());
        
        ROS_INFO_STREAM("J_x: " << base_controller.J_x.transpose());

        // begin to calculate _tau
        Eigen::VectorXd q_v = base_controller.robot_config.q_v;
        Eigen::VectorXd q_a = base_controller.robot_config.q_a;
        Eigen::VectorXd dq_v = base_controller.robot_config.dq_v;
        Eigen::VectorXd dq_a = base_controller.robot_config.dq_a;

        base_controller.compute_configuration_quantities(q_v, q_a, dq_v, dq_a); // get M，C，G，J，dJ

        base_controller.read_desired_trajectory(trajectory);

        Eigen::Matrix3d R_W_D = base_controller.robot_desired.R;
        Eigen::VectorXd p_des = base_controller.robot_desired.p;
        Eigen::VectorXd dX_des = base_controller.robot_desired.dX;
        Eigen::VectorXd ddX_des = base_controller.robot_desired.ddX;

        base_controller.compute_kinematics(q_v, q_a, dq_v, dq_a);

        Eigen::Matrix3d R = base_controller.robot_cartesian.R;
        Eigen::VectorXd p = base_controller.robot_cartesian.p;
        Eigen::VectorXd dX = base_controller.robot_cartesian.dX;

        base_controller.compute_error(R_W_D, p_des, dX_des, R, p, dX);
        base_controller.compute_cartesian_quantities(R_W_D, dX_des, ddX_des);
        base_controller.compute_damping();

        base_controller.impedance_control_torque();
        ROS_INFO_STREAM("Current tau values: " << std::fixed << std::setprecision(4) << base_controller._tau.transpose());

        // use admittance interface to transfer _tau to command.dq_v
        base_controller.admittance_interface();
        ROS_INFO_STREAM("Current vel_command: " << std::fixed << std::setprecision(4) << (base_controller.command.dq_v).transpose());
        std::cout<<"---------------------------------------------------------------------------------------------------------------"<<std::endl;

        base_command.linear.x = base_controller.command.dq_v(0);
        base_command.linear.y = base_controller.command.dq_v(1);
        base_command.angular.z = 0.1 * base_controller.command.dq_v(2);

        base_vel_pub.publish(base_command);

        signal(SIGINT, signal_callback_handler); // ensure to stop the robot when ctrl+C is pressed

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void signal_callback_handler(int signum)
{
    ROS_INFO("Whole Body Control is going to be stopped...");
    std_msgs::Float64MultiArray arm_command;
    geometry_msgs::Twist base_command;

    ros::NodeHandle n;
    ros::Publisher panda_joint_pos_pub = n.advertise<std_msgs::Float64MultiArray>("/gazebo_panda/effort_joint_position_controller/command", 0);
    ros::Publisher base_velocity_pub = n.advertise<geometry_msgs::Twist>("/darko/robotnik_base_control/cmd_vel", 0);

    base_command.linear.x = 0;
    base_command.linear.y = 0;
    base_command.angular.z = 0;

    std::vector<double> q_a_initial = {0., -0.785, 0., -2.356, 0., 1.57, 0.785};
    arm_command.data = q_a_initial;

    panda_joint_pos_pub.publish(arm_command);
    base_velocity_pub.publish(base_command);
    // Terminate program
    ros::shutdown();
}

std::vector<double> eigen_matrix_to_vector(Eigen::MatrixXd mat)
{
    std::vector<double> v(mat.size());
    Eigen::VectorXd::Map(&v[0], mat.size()) = mat;
    return v;
}