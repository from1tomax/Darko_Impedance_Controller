#include "whole_body_impedance_controller.h"
#include "tf/transform_datatypes.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "ros/ros.h"
#include <stdexcept>

/**
 * Constructor: initializes matrices sizes and assigns values to constant ones
 * @param time_step time step  = 1/ros::rate;
 */
ImpedanceControl::ImpedanceControl(double time_step=0.1) {
    
    ts = time_step;
    zeta = 0.7;

    //NUMJ_ROB = 10, which are defined in model.h
    M.resize(NUMJ_ROB, NUMJ_ROB);
    C.resize(NUMJ_ROB, NUMJ_ROB);
    G.resize(NUMJ_ROB);

    J_x.resize(6,NUMJ_ROB);
    J_x_i.resize(NUMJ_ROB,6);
    dJ.resize(6,NUMJ_ROB);
    dJ_x.resize(6,NUMJ_ROB);
    
    Kp.resize(NUMJ_ARM);
    Kp << 40.00, 40.00, 40.00, 40.00, 0.00, 00.00, 0.00;
    Kv.resize(NUMJ_ROB);
    Kv << 15.0, 15.0, 30.0, 30.00, 30.00, 30.00, 30.00, 15.00, 30.00, 30.00;

    Eigen::VectorXd Kc_coeffs(6);
    Kc_coeffs << 1000., 1000., 1000. ,10.,10.,10.;
    Kc = Eigen::MatrixXd::Identity(6,6);
    Kc.diagonal() = Kc_coeffs;  

    T_V_A <<      cos(M_PI), -sin(M_PI), 0, -0.275,
                  sin(M_PI),  cos(M_PI), 0, -0.155,
                  0,          0,         1, 0.345,
                  0,          0,         0, 1  ;
}


/**
 * initialize_kinematics: get initial position and velocity values(qa,d_qa;qv,d_qv) from /darko/robotnik_base_control/odom and /joint_states topics,
 * then compute forward kinematics and Jacobian matrix.
 * @param n ros::NodeHandle;
 */

// Retrieve base position and velosity from /darko/robotnik_base_control/odom once;
void ImpedanceControl::initialize_kinematics(ros::NodeHandle &n) {
    try{
        boost::shared_ptr<nav_msgs::Odometry const> base_odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/darko/robotnik_base_control/odom", n, ros::Duration(3));
        nav_msgs::Odometry odom_msg;

        if (base_odom_ptr == NULL) throw std::runtime_error("No vehicle odometry message received");

            odom_msg = *base_odom_ptr;

            double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
            robot_config.q_v << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw;
            robot_config.dq_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z;

            command.dq_v = robot_config.dq_v;

    }catch (const std::runtime_error& e) {
        ROS_ERROR("Initialization error: %s", e.what());
        ros::shutdown();
        throw;
    }    

    // Retrieve arm joint angle and velosity from /joint_states once;
    try{  
        boost::shared_ptr<sensor_msgs::JointState const> p_joint_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n, ros::Duration(3));
        
        if (p_joint_state_ptr == NULL) throw std::runtime_error("No arm joint state received");
            robot_config.q_a << p_joint_state_ptr->position[9], 
                                p_joint_state_ptr->position[10],
                                p_joint_state_ptr->position[11],
                                p_joint_state_ptr->position[12],
                                p_joint_state_ptr->position[13],
                                p_joint_state_ptr->position[14],
                                p_joint_state_ptr->position[15];

            robot_config.dq_a << p_joint_state_ptr->velocity[9],
                                p_joint_state_ptr->velocity[10],
                                p_joint_state_ptr->velocity[11],
                                p_joint_state_ptr->velocity[12],
                                p_joint_state_ptr->velocity[13],
                                p_joint_state_ptr->velocity[14],
                                p_joint_state_ptr->velocity[15];                        


    }catch (const std::runtime_error& e) {
        ROS_ERROR("Initialization error: %s", e.what());
        ros::shutdown();
        throw;
    }
    
    //at start the acceleration is 0
    robot_config.ddq_a<< 0, 0, 0, 0, 0, 0, 0;
    robot_config.ddq_v<< 0, 0, 0;

    //initialize desired trajectory,which is the initial position of the EE
    robot_desired.p << 2.4286, 2.6592, 1.6101;
    robot_desired.R<<  0.9955, 0.0666, -0.0678;
                       0.0666, -0.9978, -0.0033;
                       -0.0679, -0.0012, -0.9977;
    robot_desired.dX.setZero();
    robot_desired.ddX.setZero();

    //initialize all the command values equal to the initial robot_config values;
    command.dq_v = robot_config.dq_v;
    command.ddq_v = robot_config.ddq_v;

    //initialize feedforward kinematics and Jacobian matrix
    Eigen::VectorXd dq;
    // Eigen::Matrix<double, 4,4> T_W_VEH;
    // Eigen::Matrix<double, 3,3> R_W_VEH;

    dq.resize(NUMJ_ROB);
    dq << robot_config.dq_v, robot_config.dq_a;
    J_x = select_jacobian_matrix(robot_config.q_v.data(),robot_config.q_a.data(),"WHOLE_BODY");//zero jacobi matrix


    // Compute forward kinematics
    T_W_EE = forward_kinematics(robot_config.q_v,robot_config.q_a,"WHOLE_BODY");
    // T_W_VEH = forward_kinematics(robot_config.q_v, robot_config.q_a, "VEH");
    Eigen::Affine3d T_W_EE_temp(Eigen::Matrix4d::Map(T_W_EE.data()));
    // Eigen::Affine3d T_W_VEH_temp(Eigen::Matrix4d::Map(T_W_VEH.data()));

    // R_W_VEH = T_W_VEH_temp.rotation();
    // robot_config.dq_v = R_W_VEH*robot_config.dq_v;//very important, transfer the velocity of the vehicle to the world coordinate system;
    // dq << robot_config.dq_v, robot_config.dq_a;

    
    // Assign kinematics values at start 
    robot_cartesian.R = T_W_EE_temp.rotation();
    robot_cartesian.p = T_W_EE_temp.translation();
    robot_cartesian.dX = J_x*dq;//velocity of EE in cartesian space under the world coordinate system

    // /*this is a try to transfer the coordinate first 3 colum of J_x to world coordinate system;*/
    // Eigen::Matrix<double, 4,4> T_W_VEH;
    // T_W_VEH = forward_kinematics(robot_config.q_v, robot_config.q_a, "VEH");
    // Eigen::Affine3d T_W_VEH_temp(Eigen::Matrix4d::Map(T_W_VEH.data()));
    // Eigen::Matrix3d R_W_VEH = T_W_VEH_temp.rotation();

    // //transfer the first 3 columns of J_x to world coordinate system;
    // Eigen::MatrixXd R_W_VEH_aug = augment_matrix(R_W_VEH);
    // J_x.block(0,0,6,3) = R_W_VEH_aug*J_x.block(0,0,6,3);
    // robot_cartesian.dX = J_x*dq;//velocity of EE in cartesian space under the world coordinate system


       //this is a try to modify the first 3 columns of J_x
    Eigen::Matrix<double, 4,4> T_W_VEH;//transformation matrix from vehicle to world coordinate system;
    T_W_VEH = forward_kinematics(robot_config.q_v, robot_config.q_a, "VEH");
    Eigen::Affine3d T_W_VEH_temp(Eigen::Matrix4d::Map(T_W_VEH.data()));
    Eigen::Vector3d P_V_EE = T_W_EE_temp.translation() - T_W_VEH_temp.translation();
    Eigen::Matrix<double,3,3> R_W_VEH = T_W_VEH_temp.rotation();
    Eigen::Matrix<double,3,3> R_W_A =  R_W_VEH*T_V_A.block(0,0,3,3);
    Eigen::Matrix<double,6,6> R_W_A_aug = augment_matrix(R_W_A);

    Eigen::Matrix<double,6,6> Vv;
    Vv.setZero();
    Vv.topLeftCorner<3, 3>().setIdentity();
    Vv.bottomRightCorner<3, 3>().setIdentity();
    Vv.topRightCorner<3, 3>() = -calc_skewsymm(P_V_EE);

    double VEH_angle = robot_config.q_v(2);
    Eigen::Matrix<double, 6, 3> J_v;
    J_v << cos(VEH_angle), -sin(VEH_angle), 0,
           sin(VEH_angle), cos(VEH_angle), 0,
           0,               0,              0,
           0,               0,              0,
           0,               0,              0,
           0,               0,              1;
    J_x.block(0,0,6,3) = Vv*J_v;
    J_x.block(0,3,6,7) = R_W_A_aug*J_x.block(0,3,6,7);
    robot_cartesian.dX = J_x*dq;//velocity of EE in cartesian space under the world coordinate system
}

/*---- TOPIC RELATED----*/
// the following functions are conducted iteratively in the main loop of the node

void ImpedanceControl::base_config_callback(const nav_msgs::Odometry& odom_msg) {
    double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
    robot_config.q_v << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw;
    robot_config.dq_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z;
}

void ImpedanceControl::arm_config_callback(const boost::shared_ptr<const sensor_msgs::JointState> p_joint_state_ptr) {
    

    robot_config.q_a << p_joint_state_ptr->position[9], 
                        p_joint_state_ptr->position[10],
                        p_joint_state_ptr->position[11],
                        p_joint_state_ptr->position[12],
                        p_joint_state_ptr->position[13],
                        p_joint_state_ptr->position[14],
                        p_joint_state_ptr->position[15];

    robot_config.dq_a << p_joint_state_ptr->velocity[9],
                        p_joint_state_ptr->velocity[10],
                        p_joint_state_ptr->velocity[11],
                        p_joint_state_ptr->velocity[12],
                        p_joint_state_ptr->velocity[13],
                        p_joint_state_ptr->velocity[14],
                        p_joint_state_ptr->velocity[15];                        

}

/**
 * Computes Dynamics and Kinematics in configuration space. 
 *
 * @param qv vehicle position
 * @param qa arm position
 * @param dqv vehicle velocity
 * @param dqa arm joint angle velocity
 */
void ImpedanceControl::compute_configuration_quantities(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a) {

    M = select_inertia_matrix(q_v.data(),q_a.data(),"WHOLE_BODY");
    C = select_coriolis_matrix(q_v.data(),q_a.data(),dq_v.data(), dq_a.data(),"WHOLE_BODY");
    G = select_gravity_vector(q_a.data(),"WHOLE_BODY");

    J_x = select_jacobian_matrix(q_v.data(),q_a.data(),"WHOLE_BODY");
    dJ_x = calc_jacobian_derivative(q_v.data(),q_a.data(),dq_v.data(), dq_a.data(),"WHOLE_BODY");//fixme: how to modify dJ_x?

    // //modify J_x
    // Eigen::Matrix<double, 4,4> T_W_VEH;
    // T_W_VEH = forward_kinematics(robot_config.q_v, robot_config.q_a, "VEH");
    // Eigen::Affine3d T_W_VEH_temp(Eigen::Matrix4d::Map(T_W_VEH.data()));
    // Eigen::Matrix3d R_W_VEH = T_W_VEH_temp.rotation();

    // //transfer the first 3 columns of J_x to world coordinate system;
    // Eigen::MatrixXd R_W_VEH_aug = augment_matrix(R_W_VEH);
    // J_x.block(0,0,6,3) = R_W_VEH_aug*J_x.block(0,0,6,3);

}

void ImpedanceControl::compute_kinematics(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a) {

    
    Eigen::VectorXd dq;
    // Eigen::Matrix<double, 4,4> T_W_VEH;
    // Eigen::Matrix<double, 3,3> R_W_VEH;

    dq.resize(NUMJ_ROB);
    dq << dq_v, dq_a;

    // Compute forward kinematics
    T_W_EE = forward_kinematics(q_v,q_a,"WHOLE_BODY");
    // T_W_VEH = forward_kinematics(q_v, q_a, "VEH");
    Eigen::Affine3d T_W_EE_temp(Eigen::Matrix4d::Map(T_W_EE.data()));
    // Eigen::Affine3d T_W_VEH_temp(Eigen::Matrix4d::Map(T_W_VEH.data()));

    // R_W_VEH = T_W_VEH_temp.rotation();
    // robot_config.dq_v = R_W_VEH*robot_config.dq_v;//very important, transfer the velocity of the vehicle to the world coordinate system;
    // dq << robot_config.dq_v, robot_config.dq_a;
    
    // Assign kinematics values at start 
    robot_cartesian.R = T_W_EE_temp.rotation();//3*3 rotation matrix from T_W_EE， describes how EE rotates under the world coordinate system
    robot_cartesian.p = (T_W_EE_temp.translation());//position of EE in cartesian space under the world coordinate system
    robot_cartesian.dX = J_x*dq;//velocity of EE in cartesian space under the world coordinate system

    //this is a try to modify the first 3 columns of J_x
    Eigen::Matrix<double, 4,4> T_W_VEH;//transformation matrix from vehicle to world coordinate system;
    T_W_VEH = forward_kinematics(robot_config.q_v, robot_config.q_a, "VEH");
    Eigen::Affine3d T_W_VEH_temp(Eigen::Matrix4d::Map(T_W_VEH.data()));
    Eigen::Vector3d P_V_EE = T_W_EE_temp.translation() - T_W_VEH_temp.translation();
    Eigen::Matrix<double,3,3> R_W_VEH = T_W_VEH_temp.rotation();
    Eigen::Matrix<double,3,3> R_W_A = R_W_VEH*T_V_A.block(0,0,3,3);
    Eigen::Matrix<double,6,6> R_W_A_aug = augment_matrix(R_W_A);

    Eigen::Matrix<double,6,6> Vv;
    Vv.setZero();
    Vv.topLeftCorner<3, 3>().setIdentity();
    Vv.bottomRightCorner<3, 3>().setIdentity();
    Vv.topRightCorner<3, 3>() = -calc_skewsymm(P_V_EE);

    double VEH_angle = robot_config.q_v(2);
    Eigen::Matrix<double, 6, 3> J_v;
    J_v << cos(VEH_angle), -sin(VEH_angle), 0,
           sin(VEH_angle), cos(VEH_angle), 0,
           0,               0,              0,
           0,               0,              0,
           0,               0,              0,
           0,               0,              1;
    J_x.block(0,0,6,3) = Vv*J_v;
    J_x.block(0,3,6,7) = R_W_A_aug*J_x.block(0,3,6,7);
    robot_cartesian.dX = J_x*dq;//velocity of EE in cartesian space under the world coordinate system

}


void ImpedanceControl::compute_error(Eigen::Matrix3d R_W_D, Eigen::VectorXd p_des, Eigen::VectorXd dX_des, Eigen::Matrix3d R, Eigen::VectorXd p, Eigen::VectorXd dX) {

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
void ImpedanceControl::compute_cartesian_quantities(Eigen::Matrix3d R_W_D, Eigen::VectorXd dX_des, Eigen::VectorXd ddX_des) {

    Eigen::Matrix3d S_omega_des = calc_skewsymm(dX_des.tail(3));
    Eigen::Matrix3d S_alpha_des = calc_skewsymm(ddX_des.tail(3));

    v_t << dX_des.head(3)+S_omega_des*X_err.head(3),
            dX_des.tail(3);//desired velocity of EE under the world coordinate system

    dv_t << ddX_des.head(3) + S_alpha_des*X_err.head(3) + S_omega_des*dX_err.head(3), 
            ddX_des.tail(3);//desired acceleration of EE under the world coordinate system

    Eigen::Matrix<double, 6,6> Lambda_i;


    Lambda_i = J_x*M.inverse()*J_x.transpose();
    Lambda = Lambda_i.completeOrthogonalDecomposition().pseudoInverse(); //catesian mass matrix 6*6 under the world coordinate system

    J_x_i = M.inverse()*J_x.transpose()*Lambda;

    mu = J_x_i.transpose()*(C-M*J_x_i*dJ_x)*J_x_i;

}

Eigen::VectorXd ImpedanceControl::null_space_torque(Eigen::VectorXd q_v, Eigen::VectorXd q_a, Eigen::VectorXd dq_v, Eigen::VectorXd dq_a, Eigen::VectorXd q_a_des) {

    Eigen::VectorXd  tau_0(NUMJ_ROB);
    Eigen::VectorXd  tau_n(NUMJ_ROB);
    Eigen::MatrixXd J_inv (NUMJ_ROB,6);
    Eigen::MatrixXd I (NUMJ_ROB,NUMJ_ROB);
    I.setIdentity();

    tau_0.head(NUMJ_VEH) = -Kv.head(NUMJ_VEH).cwiseProduct(dq_v);
    tau_0.tail(NUMJ_ARM) = -Kp.cwiseProduct(q_a-q_a_des)-Kv.tail(NUMJ_ARM).cwiseProduct(dq_a);

    tau_n = (I-J_x.transpose()*J_x_i.transpose())*tau_0;
    return tau_n;
}// null space torque still need to be checked;


/**
 * Computes damping matrix Dc from stiffness matrix Kc to enforce desired second order system behaviour
 *
 *  Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 36-37.
 * 
 */
void ImpedanceControl::compute_damping() {

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
Eigen::VectorXd ImpedanceControl::impedance_control_torque() {
    // Eigen::VectorXd tau = G+J_x.transpose()*(Lambda*dv_t + mu*v_t - Kc*X_err - Dc*dX_err);
    // Eigen::VectorXd tau = J_x.transpose()*(- Kc*X_err - Dc*dX_err);
    // Eigen::VectorXd tau = J_x.transpose()*(-Kc*X_err);
    Eigen::VectorXd tau = G+J_x.transpose()*(-Kc*X_err- Dc*dX_err);
    return tau;
}

Eigen::VectorXd ImpedanceControl::whole_body_control_torque(Eigen::VectorXd q_a_des) {
   
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

    // tau = null_space_torque(q_v, q_a, dq_v, dq_a, q_a_des) + impedance_control_torque();
    tau = impedance_control_torque();

    // if (tau(3,0) > 87) tau(3,0) = 87;
    // if (tau(3,0) < -87) tau(3,0) = -87;
    // if (tau(4,0) > 87) tau(4,0) = 87;
    // if (tau(4,0) < -87) tau(4,0) = -87;
    // if (tau(5,0) > 87) tau(5,0) = 87;
    // if (tau(5,0) < -87) tau(5,0) = -87;
    // if (tau(6,0) > 87) tau(6,0) = 87;
    // if (tau(6,0) < -87) tau(6,0) = -87;
    // if (tau(7,0) > 12) tau(7,0) = 12;
    // if (tau(7,0) < -12) tau(7,0) = -12;
    // if (tau(8,0) > 12) tau(8,0) = 12;
    // if (tau(8,0) < -12) tau(8,0) = -12;
    // if (tau(9,0) > 12) tau(9,0) = 12;
    // if (tau(9,0) < -12) tau(9,0) = -12;
    return tau;

}

void ImpedanceControl::admittance_interface(Eigen::VectorXd tau, double dt) {


    Eigen::VectorXd ddq(NUMJ_VEH);
    Eigen::VectorXd dq(NUMJ_VEH);

    dq << command.dq_v;
    ddq << command.ddq_v;

    ddq = M.block(0,0,3,3).inverse()*tau.head(3);
    dq += ddq*dt;

    if(dq[0] > 2) dq[0] = 2;
    if(dq[0] < -2) dq[0] = -2;
    if(dq[1] > 2) dq[1] = 2;
    if(dq[1] < -2) dq[1] = -2;
    if(dq[2] > 1) dq[2] = 1;
    if(dq[2] < -1) dq[2] = -1;


    command.ddq_v = ddq;
    command.dq_v = dq;

  
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
