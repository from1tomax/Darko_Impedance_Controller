#include "model.h"
#include <iostream>

void TestKinematics() {
    int n_a = 7;
    int n_v = 3;

    // Dummy desired positions and speeds
    Eigen::VectorXd q_a(n_a);
    q_a << 0., -0.785, 0., -2.356, 0., 1.57, 0.785;
    Eigen::VectorXd q_v(n_v);
    q_v << -0.65, -2.2, 0;

    Eigen::MatrixXd J(6, 10);
    J << 1, 0, 0.155, -3.75859442669177e-17, -0.257181611791307, -4.88497843068246e-17, -0.0246661587067211, -1.31122985229289e-17, -0.107087946485338, -1.65970590020372e-36,
        0, 1, -0.581912526069448, -0.306912526069448, 3.14956637679781e-17, -0.39888875993336, 3.02073323074466e-18, -0.107070042824251, 1.31144910890532e-17, -1.35525271560688e-20,
        0, 0, 0, 0, -0.306912526069448, 0, 0.471909751092449, 0, 0.087892956017837, 0,
        0, 0, 0, 0, -1.22464679914735e-16, 0.706825181105366, 1.22464679914735e-16, -0.999999979258613, 1.22464679914735e-16, 0.000999999833333185,
        0, 0, 0, 0, -1, -8.65611195597434e-17, 1, 1.22464677374648e-16, 1, -1.22464659503937e-19,
        0, 0, 1, 1, 0, 0.7073882691672, 0, -0.000203673203695132, 0, -0.99999950000004;

    Eigen::MatrixXd J_calculated = select_jacobian_matrix(q_v.data(), q_a.data(), "WHOLE_BODY");

    std::cout << "Expected J =\n" << J << std::endl;
    std::cout << "Calculated J =\n" << J_calculated << std::endl;

    if (J.isApprox(J_calculated)) {
        std::cout << "Kinematics test passed!" << std::endl;
    } else {
        std::cout << "Kinematics test failed!" << std::endl;
    }

    Eigen::Matrix4d K;
    K << -0.707387915473095, 0.706824827692805, 0.000999999833333185, -1.23191252606945,
        0.706825181105366, 0.7073882691672, -1.22464659503937e-19, -2.355,
        -0.00070738815126905, 0.000706825063301064, -0.999999500000042, 1.28018161179131,
        0, 0, 0, 1;

    Eigen::Matrix4d K_calculated = forward_kinematics(q_v, q_a, "WHOLE_BODY");

    std::cout << "Expected K =\n" << K << std::endl;
    std::cout << "Calculated K =\n" << K_calculated << std::endl;

    if (K.isApprox(K_calculated)) {
        std::cout << "Forward kinematics test passed!" << std::endl;
    } else {
        std::cout << "Forward kinematics test failed!" << std::endl;
    }
}

void TestDynamics() {
    int n_a = 7;
    int n_v = 3;

    // Dummy desired positions and speeds
    Eigen::VectorXd q_a(n_a);
    q_a << 0., -0.785, 0., -2.356, 0., 1.57, 0.785;
    Eigen::VectorXd q_v(n_v);
    q_v << -0.65, -2.2, 0.;

    Eigen::MatrixXd M(10, 10);
    M << 217.71, 0, 1.2396, 0.10545, -3.1135, 0.065676, 0.43538, -1.316e-05, -0.069005, 0.0032553,
        0, 217.71, -8.9965, -0.20093, 3.8858e-16, -2.3312, -5.421e-17, -0.054316, 1.2143e-17, 0.0076825,
        1.2396, -8.9965, 14.303, 0.53284, -0.50516, 1.0506, 0.069052, 0.037737, -0.009033, -0.0088757,
        0.10545, -0.20093, 0.53284, 0.46124, -0.022564, 0.39932, 0.0015687, 0.022802, 0.0016628, -0.0072675,
        -3.1135, 3.8858e-16, -0.50516, -0.022564, 1.4451, -0.019399, -0.59898, -0.012804, -0.045218, 0.00038353,
        0.065676, -2.3312, 1.0506, 0.39932, -0.019399, 0.87898, -0.014317, 0.010417, 0.00059553, -0.0065528,
        0.43538, -5.421e-17, 0.069052, 0.0015687, -0.59898, -0.014317, 0.78867, 0.023469, 0.093143, -0.0013005,
        -1.316e-05, -0.054316, 0.037737, 0.022802, -0.012804, 0.010417, 0.023469, 0.027866, 0.00082125, -0.00079995,
        -0.069005, 1.2143e-17, -0.009033, 0.0016628, -0.045218, 0.00059553, 0.093143, 0.00082125, 0.032556, -0.0015701,
        0.0032553, 0.0076825, -0.0088757, -0.0072675, 0.00038353, -0.0065528, -0.0013005, -0.00079995, -0.0015701, 0.0049097;

    Eigen::MatrixXd M_calculated = select_inertia_matrix(q_v.data(), q_a.data(), "WHOLE_BODY");

    std::cout << "Expected M =\n" << M << std::endl;
    std::cout << "Calculated M =\n" << M_calculated << std::endl;

    if (M.isApprox(M_calculated, 1e-5)) {
        std::cout << "Inertia matrix test passed!" << std::endl;
    } else {
        std::cout << "Inertia matrix test failed!" << std::endl;
    }

    Eigen::VectorXd dq_a(n_a);
    dq_a << 0., -0.785, 0., -2.356, 0., 1.57, 0.785;
    Eigen::VectorXd dq_v(n_v);
    dq_v << -0.65, -2.2, 2;

    Eigen::MatrixXd C(10, 10);
    C << 0, 0, 19.2632393855691, 0.395838557819079, 4.04721374792114, 4.60670503890335, -2.70353096902093, 0.00712345262853278, -0.000170025751040767, -0.0093342975688195,
        0, 0, 3.39663546440262, 1.52345496940262, -6.22699151377214, 2.29239279121158, 0.870756435271205, 0.270950223528434, -0.138010642441697, 0.00395521230770168,
        -8.22067836980106e-17, 3.77539594444558e-17, -0.326563817144545, -0.322270139651802, 2.68393040078327, -0.432896847760782, -0.790022986051911, -0.183192647994545, 0.0807746681332026, -0.00374691886960577,
        6.75164785191029e-16, -2.14529301126526e-16, 0.0310313229792187, 0.0353250004719609, 0.344189603568151, -0.516528111207619, -0.131517666154086, -0.109785471681648, 0.0428480954531473, -0.00121241936182074,
        -4.03278105354232e-16, 3.05930284804313e-16, -2.00023824698913, -0.287815580701793, -1.30391333391444, -1.62232027137975, 0.828612619842162, 0.0310524732861739, -8.83584987103185e-05, 0.00242708824372846,
        -1.16462395283179e-16, 3.28368368223913e-16, -0.961901715465036, -0.275338060948672, 1.65879250019894, -0.892825711994263, -0.341216803445817, -0.139741069284706, 0.0451475934202696, -0.000527118358301696,
        1.30985344369994e-17, -6.98132481639315e-17, 0.338105481274673, 0.0986474615750913, 0.411620601548652, 0.306650306560299, 0.0636801125236241, -0.0340586786370316, -0.000296744017820566, 0.000205294898592384,
        2.9256816153761e-17, -1.88764260299068e-16, -0.0250659725814392, -0.00822063885953179, 0.0156887896414125, -0.0555360850129875, 0.0292236085086288, -0.0108591269876622, 0.0292789585269099, -0.00190832320482364,
        -1.0501629994177e-16, 2.65003517833205e-17, -0.0801774671897689, -0.0422245405183023, -0.0354560441058421, -0.0461513732386424, 0.0647019520784248, -0.0317982025080705, 0.000725095536980121, 0.000971078367337615,
        5.5510818770826e-17, -5.27875290338368e-17, 0.00687356404291103, 0.00270154941378384, -0.00161564760440182, 0.00827371594561118, -0.00229260989851095, 0.0119095721474224, -0.00159554633098499, 0;

    Eigen::MatrixXd C_calculated = select_coriolis_matrix(q_v.data(), q_a.data(), dq_v.data(), dq_a.data(), "WHOLE_BODY");

    std::cout << "Expected C =\n" << C << std::endl;
    std::cout << "Calculated C =\n" << C_calculated << std::endl;

    if (C.isApprox(C_calculated)) {
        std::cout << "Coriolis matrix test passed!" << std::endl;
    } else {
        std::cout << "Coriolis matrix test failed!" << std::endl;
    }

    Eigen::VectorXd G(10);
    G << 0., 0., 0., 0., -1.78161532593602, -0.643548502755121, 18.5677222149051, 0.63366294717851, 1.6924518378428, 3.19239555533241e-05;

    Eigen::VectorXd G_calculated = select_gravity_vector(q_a.data(), "WHOLE_BODY");

    std::cout << "Expected G =\n" << G << std::endl;
    std::cout << "Calculated G =\n" << G_calculated << std::endl;

    if (G.isApprox(G_calculated)) {
        std::cout << "Gravity vector test passed!" << std::endl;
    } else {
        std::cout << "Gravity vector test failed!" << std::endl;
    }
}

int main() {
    TestKinematics();
    TestDynamics();
    return 0;
}
