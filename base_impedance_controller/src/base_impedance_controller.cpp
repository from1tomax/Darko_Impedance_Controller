#include "base_impedance_controller.h"
#include "tf/transform_datatypes.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "ros/ros.h"
#include "model.h"

/**
 * Constructor: initializes matrices sizes and assigns values to constant ones
 *
 * @param choose_body Robot part to be controlled: "WHOLE_BODY", "EE", "VEH"
 * @param choose_ts control rate
 */
BaseImpedanceControl::BaseImpedanceControl(std::string choose_body, double choose_ts)
{

    ts = choose_ts;
    body = resolve_rigid_body(choose_body);

    M.resize(NUMJ_VEH, NUMJ_VEH);
    C.resize(NUMJ_VEH, NUMJ_VEH);
    G.resize(NUMJ_VEH);
    J.resize(6, NUMJ_VEH);
    _tau.resize(NUMJ_VEH);

    J_x.resize(6, NUMJ_VEH);
    J_x_old.resize(6, NUMJ_VEH);
    J_x = Eigen::MatrixXd::Zero(6, NUMJ_VEH); // Initialize value to get a reasonable J_x_old at the beginning
    J_x_i.resize(NUMJ_VEH, 6);

    dJ.resize(6, NUMJ_VEH);
    dJ_x.resize(6, NUMJ_VEH);

    command.dq_v = Eigen::VectorXd::Zero(NUMJ_VEH);
    command.ddq_v = Eigen::VectorXd::Zero(NUMJ_VEH);

    zeta = 0.7; // damping factor!values between[0,1]
}

void BaseImpedanceControl::initialize_kinematics(ros::NodeHandle &n)
{

    boost::shared_ptr<nav_msgs::Odometry const> base_odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/darko/robotnik_base_control/odom", n, ros::Duration(3));
    boost::shared_ptr<sensor_msgs::JointState const> p_joint_state_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n, ros::Duration(3));

    nav_msgs::Odometry odom_msg;

    // Retrieve robot configuration from topics once
    if (base_odom_ptr == nullptr)
        ROS_INFO("No vehicle odometry message received");
    else
    {
        odom_msg = *base_odom_ptr;
        double yaw = tf::getYaw(odom_msg.pose.pose.orientation);

        robot_config.q_v << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw;
        robot_config.dq_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z;
    }

    if (p_joint_state_msg == nullptr)
        ROS_INFO("No arm joint state received");
    else
    {
        robot_config.q_a << p_joint_state_msg->position[6],
            p_joint_state_msg->position[7],
            p_joint_state_msg->position[8],
            p_joint_state_msg->position[9],
            p_joint_state_msg->position[10],
            p_joint_state_msg->position[11],
            p_joint_state_msg->position[12];
        robot_config.dq_a << 0., 0., 0., 0., 0., 0., 0.; 

    }

    std::vector<double> Kc_coeffs_vec(6);
    if (!n.getParam("/base_impedance_controller_node/Kc_coeffs_vec", Kc_coeffs_vec)) {
    ROS_WARN("No Kc_coeffs_vec parameter found, using default values.");
    Kc_coeffs_vec = {50, 50, 0, 0, 0, 50};  
    }
    Eigen::VectorXd Kc_coeffs = Eigen::VectorXd::Map(Kc_coeffs_vec.data(), Kc_coeffs_vec.size());

    Kc = Eigen::MatrixXd::Identity(6, 6);
    Kc.diagonal() = Kc_coeffs;

    _tau << 0.0, 0.0, 0.0;
    robot_config.ddq_v << 0.0, 0.0, 0.0;
    Eigen::Matrix4d K;

    // Compute forward kinematics for VEH

    K = forward_kinematics(robot_config.q_v, robot_config.q_a, "VEH");
    J = select_jacobian_matrix(robot_config.q_v.data(), robot_config.q_a.data(), "VEH");

    // Assign kinematics values at start
    robot_cartesian.R = K.topLeftCorner(3, 3);
    robot_cartesian.p << K.coeff(0, 3), K.coeff(1, 3), K.coeff(2, 3);
    robot_cartesian.dX = J * robot_config.dq_v;
    robot_cartesian.ddX << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;//fixit：how to generate ddx from dX = J * robot_config.dq_v? ans: ddx = J * robot_config.ddq_v + dJ * robot_config.dq_v. but we do not know ddq_v yet.
    // initialize robot_cartesian_initial
    robot_cartesian_initial.R = K.topLeftCorner(3, 3);
    robot_cartesian_initial.p << K.coeff(0, 3), K.coeff(1, 3), K.coeff(2, 3);
    robot_cartesian_initial.dX = J * robot_config.dq_v;
    robot_cartesian_initial.ddX << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // J_x = augment_matrix(robot_cartesian.R.transpose()) * J;
    J_x = J;//under the base coordinate system
}


void BaseImpedanceControl::base_config_callback(const nav_msgs::Odometry &odom_msg)
{

    double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
    robot_config.q_v << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw;
    robot_config.dq_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z;
}

void BaseImpedanceControl::arm_config_callback(const boost::shared_ptr<const sensor_msgs::JointState> p_joint_state_msg)
{

    // Retrieve new values
    robot_config.q_a << p_joint_state_msg->position[6],
        p_joint_state_msg->position[7],
        p_joint_state_msg->position[8],
        p_joint_state_msg->position[9],
        p_joint_state_msg->position[10],
        p_joint_state_msg->position[11],
        p_joint_state_msg->position[12];

    robot_config.dq_a = Eigen::VectorXd::Zero(NUMJ_ARM);
}


/* calculate M, C, G, J, dJ every time step, which all under the base coordinate system */
void BaseImpedanceControl::compute_configuration_quantities(Eigen::VectorXd q_v, Eigen::VectorXd q_a,
                                                        Eigen::VectorXd dq_v, Eigen::VectorXd dq_a)
{

    Eigen::Matrix<double, 10, 10> M_temp;
    Eigen::Matrix<double, 10, 10> C_temp;

    M_temp = select_inertia_matrix(q_v.data(), q_a.data(), "WHOLE_BODY");
    M = M_temp.topLeftCorner(3, 3);
    C_temp = select_coriolis_matrix(q_v.data(), q_a.data(), dq_v.data(), dq_a.data(), "WHOLE_BODY");
    C = C_temp.topLeftCorner(3, 3);
    G = select_gravity_vector(q_a.data(), "VEH");

    J = select_jacobian_matrix(q_v.data(), q_a.data(), "VEH");
    dJ = calc_jacobian_derivative(q_v.data(), q_a.data(), dq_v.data(), dq_a.data(), "VEH");

}


void BaseImpedanceControl::compute_kinematics(Eigen::VectorXd q_v, Eigen::VectorXd q_a,
                                          Eigen::VectorXd dq_v, Eigen::VectorXd dq_a)
{

    Eigen::Matrix4d K;

    // Compute forward kinematics
    K = forward_kinematics(q_v, q_a, "VEH");

    // Assign kinematics values at start
    robot_cartesian.R = K.topLeftCorner(3, 3);
    robot_cartesian.p << K.coeff(0, 3), K.coeff(1, 3), K.coeff(2, 3);
    robot_cartesian.dX = J * dq_v;
    robot_cartesian.ddX << 0.0, 0.0, 0.0;
}


void BaseImpedanceControl::read_desired_trajectory(CartesianSpace trajectory)
{
    robot_desired.R = trajectory.R;
    robot_desired.p = trajectory.p;
    robot_desired.dX = trajectory.dX;
    robot_desired.ddX = trajectory.ddX;
}



/*计算出四元数误差qt_err， 位置误差X_err， 速度误差*/
void BaseImpedanceControl::compute_error(Eigen::Matrix3d R_W_D, Eigen::VectorXd p_des, Eigen::VectorXd dX_des,
                                     Eigen::Matrix3d R, Eigen::VectorXd p, Eigen::VectorXd dX)
{

    Eigen::Quaterniond qt_des(R_W_D);
    Eigen::Quaterniond qt(R);

    qt_des.normalize();
    qt.normalize();

    Eigen::Quaterniond qt_err = qt_des.inverse() * qt;
    // std::cout << std::setprecision (15) << "p_inside_comp_error = \n" << p << std::endl;
    // std::cout << std::setprecision (15) << "p_des_inside_comp_error = \n" << p_des << std::endl;
    // std::cout << std::setprecision (15) << "R_W_D_inside_comp_error = \n" << R_W_D << std::endl;

    if (qt_err.w() < 0)
    {
        qt_err.w() = -qt_err.w();
        qt_err.x() = -qt_err.x();
        qt_err.y() = -qt_err.y();
        qt_err.z() = -qt_err.z();
    }
    // X_err << R_W_D.transpose() * (p - p_des),
    //     2 * qt_err.x(),
    //     2 * qt_err.y(),
    //     2 * qt_err.z();
    X_err << R.transpose() * (p - p_des),
        2 * qt_err.x(),
        2 * qt_err.y(),
        2 * qt_err.z();

    // std::cout << std::setprecision (15) << "X_err_inside_comp_error = \n" << X_err << std::endl;

    // Eigen::Matrix<double, 6, 6> R_D_W_aug = augment_matrix(R_W_D.transpose());
    // Eigen::Matrix<double, 6, 6> R_D_W_aug = augment_matrix(R.transpose());
    // dX_err << dX.head(3) - dX_des.head(3) - calc_skewsymm(dX_des.tail(3)) * (p - p_des),
    //     dX.tail(3) - dX_des.tail(3);

    Eigen::Matrix<double, 6, 6> R_D_W_aug = augment_matrix(R.transpose());
    dX_err << dX.head(3) - dX_des.head(3) - calc_skewsymm(dX.tail(3)) * (p - p_des),
              dX.tail(3) - dX_des.tail(3);

    // dX_err = R_D_W_aug * dX_err;
}


/*
 * Computes Dynamics and Kinematics in cartesian space. Cartesian error and Jacobian are redefined through
 * the concepts of rotational and translational stiffness. The orientation representation adopted here is
 * quaternions.
 * Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 41-45.
 * Computes the mass matrix Lambda_i and its pseudoinverse Lambda, as well as the Coriolis matrix mu in cartesian space.
 * Calculates the desired velocity v_t and acceleration dv_t.
 *
Finished  <<< whole_body_controller
 * @param R_W_D   Rotation between world frame and desired frame
 * @param dX_des  Desired twist
 * @param ddX_des Desired Acceleration
 */
void BaseImpedanceControl::compute_cartesian_quantities(Eigen::Matrix3d R_W_D, Eigen::VectorXd dX_des,
                                                    Eigen::VectorXd ddX_des)
{

    Eigen::Matrix<double, 6, 6> R_D_W_aug = augment_matrix(R_W_D.transpose());
    Eigen::Matrix<double, 6, 1> dv_t_temp;

    J_x = J;

    Eigen::Matrix3d S_omega_des = calc_skewsymm(dX_des.tail(3));
    Eigen::Matrix3d S_alpha_des = calc_skewsymm(ddX_des.tail(3));

    Eigen::Matrix<double, 6, 1> a;
    a << dX_des.head(3) + S_omega_des * X_err.head(3),
        dX_des.tail(3);

    // v_t = R_D_W_aug * a;
    v_t = a;

    dv_t_temp << ddX_des.head(3) + S_alpha_des * X_err.head(3) + S_omega_des * dX_err.head(3),
        ddX_des.tail(3);

    // dv_t = R_D_W_aug * (-augment_matrix(S_omega_des) * a + dv_t_temp);
    dv_t = (-augment_matrix(S_omega_des) * a + dv_t_temp);

    Eigen::Matrix<double, 6, 6> Lambda_i;

    Lambda_i = J_x * M.inverse() * J_x.transpose();
    // std::cout << "Lambda_i = \n" << Lambda_i << std::endl;
    Lambda = Lambda_i.completeOrthogonalDecomposition().pseudoInverse();

    // dJ_x = R_D_W_aug * (dJ - augment_matrix(S_omega_des) * J);
    dJ_x = (dJ - augment_matrix(S_omega_des) * J);

    // std::cout << "dJ_x = \n" << dJ_x << std::endl;
    J_x_i = M.inverse() * J_x.transpose() * Lambda;

    mu = J_x_i.transpose() * (C - M * J_x_i * dJ_x) * J_x_i;
}


/**
 * Computes damping matrix Dc from stiffness matrix Kc to enforce desired second order system behaviour
 *
 *  Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 36-37.
 *
 */
void BaseImpedanceControl::compute_damping()
{

    Eigen::Matrix3d Lambda_3 = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d Kc_3 = Eigen::Matrix3d::Zero(3, 3);
    Lambda_3.topLeftCorner(2, 2) = Lambda.topLeftCorner(2, 2);
    Lambda_3.block(2, 0, 1, 2) = Lambda.block(5, 0, 1, 2);
    Lambda_3.block(0, 2, 2, 1) = Lambda.block(0, 5, 2, 1);
    Lambda_3(2, 2) = Lambda(5, 5);

    Kc_3(0, 0) = Kc.diagonal()[0];
    Kc_3(1, 1) = Kc.diagonal()[1];
    Kc_3(2, 2) = Kc.diagonal()[5];

    Eigen::Matrix<double, 3, 3> A_3;
    A_3 = Lambda_3.sqrt();
    Eigen::Matrix<double, 3, 3> Kc_3_1;
    Kc_3_1 = Kc_3.sqrt();
    Eigen::Matrix<double, 3, 3> D0;
    D0 = zeta * Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix3d Dc_3;
    Dc_3 = A_3 * D0 * Kc_3_1 + Kc_3_1 * D0 * A_3;

    Dc = Eigen::Matrix<double, 6, 6>::Zero(6, 6);
    Dc.topLeftCorner(2, 2) = Dc_3.topLeftCorner(2, 2);
    Dc.block(5, 0, 1, 2) = Dc_3.block(2, 0, 1, 2);
    Dc.block(0, 5, 2, 1) = Dc_3.block(0, 2, 2, 1);
    Dc(5, 5) = Dc_3(2, 2);
}

/**
 * PD impedance controller with gravity compensation
 *  Reference: Cartesian Impedance Control of Redundant and Flexible-Joint Robots, C. Ott 2005, page 41-44.
 *
 * @return torque contribute for attaining desired impedance
 */
void BaseImpedanceControl::impedance_control_torque()
{
    _tau = G+J_x.transpose() * (Lambda * (-dv_t) + mu * (-v_t) - Kc * X_err - Dc * dX_err);
    // _tau = G + J_x.transpose()*(-Kc * X_err- Dc * dX_err) ;
}

void BaseImpedanceControl::whole_body_control_torque(CartesianSpace trajectory)
{

    Eigen::VectorXd q_v = robot_config.q_v;
    Eigen::VectorXd q_a = robot_config.q_a;
    Eigen::VectorXd dq_v = robot_config.dq_v;
    Eigen::VectorXd dq_a = robot_config.dq_a;

    compute_configuration_quantities(q_v, q_a, dq_v, dq_a); // to get M,C,G,J,dJ

    read_desired_trajectory(trajectory);

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

    impedance_control_torque();
}


void BaseImpedanceControl::admittance_interface()
{

    command.ddq_v = M.inverse() * (_tau);
    command.dq_v += command.ddq_v * 0.1;
}

/* Utility functions */
Eigen::Matrix3d calc_skewsymm(Eigen::Vector3d v)
{

    Eigen::Matrix3d S;

    S << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return S;
}

Eigen::MatrixXd augment_matrix(Eigen::Matrix3d M)
{

    Eigen::Matrix<double, 6, 6> aug = Eigen::MatrixXd::Zero(6, 6);
    aug.topLeftCorner(3, 3) = M;
    aug.bottomRightCorner(3, 3) = M;

    return aug;
}

