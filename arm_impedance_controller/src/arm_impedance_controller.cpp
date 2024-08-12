#include "arm_impedance_controller.h"
#include "tf/transform_datatypes.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "ros/ros.h"

/**
 * Constructor: initializes matrices sizes and assigns values to constant ones
 * @param choose_ts control rate
 */
ArmImpedanceControl::ArmImpedanceControl(double choose_ts) { 

    ts = choose_ts;
    M.resize(NUMJ_ARM, NUMJ_ARM);
    C.resize(NUMJ_ARM, NUMJ_ARM);
    G.resize(NUMJ_ARM);

    J_x.resize(6,NUMJ_ARM);
    J_x = Eigen::MatrixXd::Zero(6,NUMJ_ARM);
    J_x_i.resize(NUMJ_ARM,6);
    dJ_x.resize(6,NUMJ_ARM);

    command.dq_v = Eigen::VectorXd::Zero(NUMJ_VEH);
    command.ddq_v = Eigen::VectorXd::Zero(NUMJ_VEH);
    command.dq_a = Eigen::VectorXd::Zero(NUMJ_ARM);
    command.ddq_a = Eigen::VectorXd::Zero(NUMJ_ARM);
            
    Kp.resize(NUMJ_ARM);
    Kp << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0;
    Kv.resize(NUMJ_ARM);
    Kv << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;

    Eigen::VectorXd Kc_coeffs(6);
    Kc_coeffs << 1000., 1000., 1000. ,10.,10.,10.; //1000.,1000.,1000.,1.,1.,2.;
    Kc = Eigen::MatrixXd::Identity(6,6);
    Kc.diagonal() = Kc_coeffs;
        
    zeta = 0.7;
}

void ArmImpedanceControl::initialize_kinematics(ros::NodeHandle &n) {

    boost::shared_ptr<nav_msgs::Odometry const> base_odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/darko/robotnik_base_control/odom", n, ros::Duration(3));
    boost::shared_ptr<sensor_msgs::JointState const> p_joint_state_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n, ros::Duration(3));

    nav_msgs::Odometry odom_msg;

    // Retrieve robot configuration from topics once
    if (base_odom_ptr == NULL) ROS_INFO("No vehicle odometry message received");
    else {
        odom_msg = *base_odom_ptr;
        double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
        robot_config.q_v << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw;
        robot_config.dq_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z;

        command.q_v = robot_config.q_v;
    }

    if (p_joint_state_msg == NULL) ROS_INFO("No arm joint state received");
    else {
        robot_config.q_a << p_joint_state_msg->position[8], 
                                    p_joint_state_msg->position[9],
                                    p_joint_state_msg->position[10],
                                    p_joint_state_msg->position[11],
                                    p_joint_state_msg->position[12],
                                    p_joint_state_msg->position[13],
                                    p_joint_state_msg->position[14];
        robot_config.dq_a << p_joint_state_msg->velocity[8], 
                                    p_joint_state_msg->velocity[9],
                                    p_joint_state_msg->velocity[10],
                                    p_joint_state_msg->velocity[11],
                                    p_joint_state_msg->velocity[12],
                                    p_joint_state_msg->velocity[13],
                                    p_joint_state_msg->velocity[14];

        command.q_a = robot_config.q_a;

    }
    //initilize impedance stiffness matrix
    std::vector<double> Kc_coeffs_vec;
    if(!n.getParam("/arm_impedance_controller_node/Kc_coeffs_vec", Kc_coeffs_vec)) {
        ROS_ERROR("Failed to get Kc_coeffs from parameter server, using default values: ");
        Kc_coeffs_vec = {1000., 1000., 1000. ,10.,10.,10.};
    }
    Eigen::Map<Eigen::VectorXd> Kc_coeffs(Kc_coeffs_vec.data(), Kc_coeffs_vec.size());
    Kc = Eigen::MatrixXd::Identity(6,6);
    Kc.diagonal() = Kc_coeffs;

    
    // Compute forward kinematics
    T_W_EE = forward_kinematics(robot_config.q_v,robot_config.q_a,"EE");
    J_x = select_jacobian_matrix(robot_config.q_v.data(),robot_config.q_a.data(),"EE");//jacobi matrix under world coordinate system
   
    // Assign kinematics values at start 
    robot_cartesian.R = T_W_EE.topLeftCorner(3,3);
    robot_cartesian.p << T_W_EE.coeff(0,3), T_W_EE.coeff(1,3), T_W_EE.coeff(2,3);
    robot_cartesian.dX = J_x*robot_config.dq_a;
}

/*---- TOPIC RELATED----*/
//get current vehicle state
void ArmImpedanceControl::base_config_callback(const nav_msgs::Odometry& odom_msg) {
    double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
    robot_config.q_v << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw;
    robot_config.dq_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z;
}

//get current arm state
void ArmImpedanceControl::arm_config_callback(const boost::shared_ptr<const sensor_msgs::JointState> p_joint_state_msg) {  
    robot_config.q_a << p_joint_state_msg->position[8], 
                        p_joint_state_msg->position[9],
                        p_joint_state_msg->position[10],
                        p_joint_state_msg->position[11],
                        p_joint_state_msg->position[12],
                        p_joint_state_msg->position[13],
                        p_joint_state_msg->position[14];

    robot_config.dq_a << p_joint_state_msg->velocity[8],
                        p_joint_state_msg->velocity[9],
                        p_joint_state_msg->velocity[10],
                        p_joint_state_msg->velocity[11],
                        p_joint_state_msg->velocity[12],
                        p_joint_state_msg->velocity[13],
                        p_joint_state_msg->velocity[14];
}

/**
 * Computes Dynamics and Kinematics in configuration space. 
 *
 * @param qv vehicle configuration
 * @param qa arm configuration
 * @param dqv vehicle configuration velocities
 * @param dqa arm joint velocites
 */
void ArmImpedanceControl::compute_configuration_quantities(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a){

    M = select_inertia_matrix(q_v.data(),q_a.data(),"EE");
    C = select_coriolis_matrix(q_v.data(),q_a.data(),dq_v.data(), dq_a.data(),"EE");
    G = select_gravity_vector(q_a.data(),"EE");

    J_x = select_jacobian_matrix(q_v.data(),q_a.data(),"EE");
    dJ_x= calc_jacobian_derivative(q_v.data(), q_a.data(), dq_v.data(), dq_a.data(),"EE");
}

void ArmImpedanceControl::compute_kinematics(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a) {

  // Compute forward kinematics
    T_W_EE = forward_kinematics(q_v,q_a,"EE");
    
    robot_cartesian.R = T_W_EE.topLeftCorner(3,3);
    robot_cartesian.p << T_W_EE.coeff(0,3), T_W_EE.coeff(1,3), T_W_EE.coeff(2,3);
    robot_cartesian.dX = J_x*robot_config.dq_a;

}


void ArmImpedanceControl::compute_error(Eigen::Matrix3d R_W_D, Eigen::VectorXd p_des, Eigen::VectorXd dX_des, Eigen::Matrix3d R, Eigen::VectorXd p, Eigen::VectorXd dX) {

    Eigen::Quaterniond qt_des(R_W_D);
    Eigen::Quaterniond qt(R);

    qt_des.normalize();
    qt.normalize();

    if(qt_des.coeffs().dot(qt.coeffs()) < 0.0) {
        qt.coeffs() << -qt.coeffs();
    }

    Eigen::Quaterniond qt_err(qt.inverse() * qt_des);
    qt_err.normalize();
    X_err.head(3) << (p-p_des);
    X_err.tail(3) <<qt_err.x(),
                    qt_err.y(),
                    qt_err.z();
    // this X_err.tail(3) error is under the EE coordinate system;
    X_err.tail(3) = -R*X_err.tail(3);
    //now the X_err is under the World coordinate system;

    dX_err << dX.head(3)-dX_des.head(3)-calc_skewsymm(dX_des.tail(3))*(p-p_des),
                                                    dX.tail(3)-dX_des.tail(3);
    //now dX_err is under the World coordinate system;
}

/**
 * Computes Dynamics and Kinematics in cartesian space. Cartesian error and Jacobian are redefined through 
 * the concepts of rotational and translational stiffness. The orientation representation adopted here is 
 * quaternions.
 * Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 41-45.
 *                                                                                                                                      
Finished  <<< whole_body_controller              
 * @param R_W_D   Rotation between world frame and desired frame
 * @param dX_des  Desired twist
 * @param ddX_des Desired Acceleration 
 * 
 */
void ArmImpedanceControl::compute_cartesian_quantities(Eigen::Matrix3d R_W_D, Eigen::VectorXd dX_des, Eigen::VectorXd ddX_des){

    Eigen::Matrix3d S_omega_des = calc_skewsymm(dX_des.tail(3));
    Eigen::Matrix3d S_alpha_des = calc_skewsymm(ddX_des.tail(3));

    v_t << dX_des.head(3)+S_omega_des*X_err.head(3),
            dX_des.tail(3);
    dv_t << ddX_des.head(3) + S_alpha_des*X_err.head(3) + S_omega_des*dX_err.head(3), 
            ddX_des.tail(3);

    Eigen::Matrix<double, 6,6> Lambda_i;


    Lambda_i = J_x*M.inverse()*J_x.transpose();
    Lambda = Lambda_i.completeOrthogonalDecomposition().pseudoInverse(); 

    J_x_i = M.inverse()*J_x.transpose()*Lambda;

    mu = J_x_i.transpose()*(C-M*J_x_i*dJ_x)*J_x_i;

}

Eigen::VectorXd ArmImpedanceControl::null_space_torque(Eigen::VectorXd q_a, Eigen::VectorXd dq_a, Eigen::VectorXd q_a_des) {

    Eigen::VectorXd  tau_0(NUMJ_ARM);
    Eigen::VectorXd  tau_n(NUMJ_ARM);
    Eigen::MatrixXd  I (NUMJ_ARM,NUMJ_ARM);
    I.setIdentity();
        
    // tau_0 = -M*(Kp.cwiseProduct(q_a-q_a_des)+Kv.cwiseProduct(dq_a));
    tau_0 = -(Kp.cwiseProduct(q_a-q_a_des)+Kv.cwiseProduct(dq_a));
    //Project into nullspace
    tau_n = (I-J_x.transpose()*J_x_i.transpose())*tau_0;
    // std::cout << "tau_n = \n" << tau_n << std::endl;
    return tau_n;
}


/**
 * Computes damping matrix Dc from stiffness matrix Kc to enforce desired second order system behaviour
 *
 *  Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 36-37.
 * 
 */
void ArmImpedanceControl::compute_damping(){
    Eigen::Matrix<double,6,6> A;
    A = Lambda.sqrt();
    Eigen::Matrix<double,6,6> Kc_1;
    Kc_1 = Kc.sqrt();
    Eigen::Matrix<double,6,6> D0;
    D0 = zeta*Eigen::MatrixXd::Identity(6,6);

    Dc = A*D0*Kc_1 + Kc_1*D0*A;

}

/**
 * PD impedance controller with gravity compensation
 *  Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 41-44.
 * 
 * @return torque contribute for attaining desired impedance
 */
Eigen::VectorXd ArmImpedanceControl::impedance_control_torque() {
    Eigen::VectorXd tau = G+J_x.transpose()*(Lambda*(-dv_t) + mu*(-v_t) - Kc*X_err - Dc*dX_err);
    // Eigen::VectorXd tau = G+J_x.transpose()*(- Kc*X_err - Dc*dX_err);
    return tau;
}

Eigen::VectorXd ArmImpedanceControl::whole_body_control_torque(Eigen::VectorXd q_a_des) {
  
    Eigen::VectorXd tau;
    Eigen::VectorXd q_v = robot_config.q_v;
    Eigen::VectorXd q_a = robot_config.q_a;
    Eigen::VectorXd dq_v = robot_config.dq_v;
    Eigen::VectorXd dq_a = robot_config.dq_a;
    
    compute_configuration_quantities(q_v, q_a, dq_v, dq_a);

    Eigen::Matrix3d R_W_D = robot_desired.R;
    Eigen::VectorXd p_des = robot_desired.p;
    Eigen::VectorXd dX_des = robot_desired.dX;
    Eigen::VectorXd ddX_des = robot_desired.ddX;

    compute_kinematics(q_v, q_a, dq_v, dq_a);

    Eigen::Matrix3d R = robot_cartesian.R;
    Eigen::VectorXd p = robot_cartesian.p;
    Eigen::VectorXd dX = robot_cartesian.dX;
    
    compute_error(R_W_D, p_des, dX_des, R, p, dX);
    compute_cartesian_quantities(R_W_D, dX_des, ddX_des);
    compute_damping();


    tau = null_space_torque(q_a, dq_a, q_a_des) + impedance_control_torque();
    // tau = impedance_control_torque();

    // if (tau(0,0) > 87) tau(0,0) = 87;
    // if (tau(0,0) < -87) tau(0,0) = -87;
    // if (tau(1,0) > 87) tau(1,0) = 87;
    // if (tau(1,0) < -87) tau(1,0) = -87;
    // if (tau(2,0) > 87) tau(2,0) = 87;
    // if (tau(2,0) < -87) tau(2,0) = -87;
    // if (tau(3,0) > 87) tau(3,0) = 87;
    // if (tau(3,0) < -87) tau(3,0) = -87;
    // if (tau(4,0) > 12) tau(4,0) = 12;
    // if (tau(4,0) < -12) tau(4,0) = -12;
    // if (tau(5,0) > 12) tau(5,0) = 12;
    // if (tau(5,0) < -12) tau(5,0) = -12;
    // if (tau(6,0) > 12) tau(6,0) = 12;
    // if (tau(6,0) < -12) tau(6,0) = -12;

    return tau;
}


/* Utility functions */
Eigen::Matrix3d calc_skewsymm(Eigen::Vector3d v) {
  
  Eigen::Matrix3d S;

  S <<   0., -v(2),    v(1),
       v(2),    0.,   -v(0),
      -v(1),  v(0),      0.;
    
  return S;
}

Eigen::MatrixXd augment_matrix(Eigen::Matrix3d M) {
  
  Eigen::Matrix<double, 6,6> aug = Eigen::MatrixXd::Zero(6,6);
  aug.topLeftCorner(3,3) = M;
  aug.bottomRightCorner(3,3) = M;

  return aug;
}

/**
 * Computes the time derivative of a matrix. The method in use is the simple finite difference
 * 
 * @param A     matrix at current instant
 * @param A_old matrix at previous sample
 * @param ts    sampling time
*/
Eigen::MatrixXd time_derivative(Eigen::MatrixXd A, Eigen::MatrixXd A_old, double ts) {
    Eigen::MatrixXd A_d;
    A_d = (A - A_old)/ts;
    return A_d;
}

Eigen::VectorXd time_integration(Eigen::VectorXd v_int0, Eigen::VectorXd v, Eigen::VectorXd v_old, double ts) {
    Eigen::VectorXd v_int;
    v_int = (v+v_old)/2*ts + v_int0;
    return v_int;
}


