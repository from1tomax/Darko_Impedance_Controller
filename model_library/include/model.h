/*
 *   model - Dynamic Robot Model of the RB-KAIROS+ platfor
 *   Owner 2023 Alessandro De Toni <alessandro.detoni98@gmail.com>
 *
 */

#ifndef MODEL_H
#define MODEL_H

/* Include files */
#include "rtwtypes.h"
#include <eigen3/Eigen/Eigen>
#include <cstddef>
#include <cstdlib>

/* Macros definitions */
#define NUMJ_VEH 3
#define NUMJ_ARM 7

const int NUMJ_ROB = NUMJ_ARM+NUMJ_VEH;

enum rigid_bodies {
     VEH,
     EE,
     WHOLE_BODY
 };

// struct ConfigurationSpace {
//     Eigen::Matrix<double,NUMJ_VEH,1> q_v;
//     Eigen::Matrix<double,NUMJ_ARM,1> q_a;
//     Eigen::Matrix<double,NUMJ_VEH,1> dq_v;
//     Eigen::Matrix<double,NUMJ_ARM,1> dq_a;
//     Eigen::Matrix<double,NUMJ_VEH,1> ddq_v;
//     Eigen::Matrix<double,NUMJ_ARM,1> ddq_a;
    

//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };


// struct CartesianSpace {
//     Eigen::Matrix<double,3,1> p;
//     Eigen::Matrix3d R;
//     Eigen::Matrix<double,6,1> dX;
//     Eigen::Matrix<double,6,1> ddX;

//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };



//extern int SIMJOINTS[NUMJ_MOD];

int resolve_rigid_body(std::string rigid_body);

Eigen::Matrix4d forward_kinematics(Eigen::VectorXd q_v, Eigen::VectorXd q_a, std::string rb_location);

Eigen::Matrix4d get_T_A_EE(const double qa[NUMJ_ARM]);

Eigen::MatrixXd select_jacobian_matrix(const double qv[NUMJ_VEH], const double qa[NUMJ_ARM], std::string rb_motion);

Eigen::MatrixXd get_J(const double qv[NUMJ_VEH], const double qa[NUMJ_ARM]);//得到6*10的雅克比矩阵

Eigen::MatrixXd select_gravity_vector(const double qa[NUMJ_ARM], std::string rb_motion);

Eigen::VectorXd get_G(const double qa[7]);//得到一个10*1的重力向量

Eigen::MatrixXd select_inertia_matrix(const double qv[NUMJ_VEH], const double qa[NUMJ_ARM], std::string rb_motion);

Eigen::MatrixXd get_M(const double qv[NUMJ_VEH], const double qa[NUMJ_ARM]);//得到10*10质量矩阵

Eigen::Matrix3d get_Mv(const double qv[NUMJ_VEH]);//得到3*3质量矩阵

Eigen::MatrixXd select_coriolis_matrix(const double qv[NUMJ_VEH], const double qa[NUMJ_ARM], const double dqv[NUMJ_VEH], const double dqa[NUMJ_ARM], std::string rb_motion);

Eigen::MatrixXd get_C(const double qv[NUMJ_VEH], const double qa[NUMJ_ARM], const double dqv[NUMJ_VEH], const double dqa[NUMJ_ARM]);

Eigen::Matrix3d get_Cv(const double qv[NUMJ_VEH], const double qdv[NUMJ_VEH]);//得到3*3科式矩阵

Eigen::MatrixXd calc_jacobian_derivative(const double qv[3], const double qa[7], const double qv_dot[3], const double qa_dot[7], std::string rb_motion);

void get_J_gradient(double qa1, double qa2, double qa3, double qa4, double qa5, double qa6, double qv3, double J_grad[600]);

#endif
