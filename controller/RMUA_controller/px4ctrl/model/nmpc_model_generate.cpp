#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

#define CODE_GEN true
int main()
{
    /***********************在此处填充完整的动力学模型 *********************/

    USING_NAMESPACE_ACADO
    DifferentialState ksi_x;
    DifferentialState ksi_y;
    DifferentialState ksi_z;                  // 状态变量
    DifferentialState ksi_vx, ksi_vy, ksi_vz; // 状态变量
    DifferentialState qw, qx, qy, qz;         // 状态变量
    // DifferentialState omega_Bx, omega_By, omega_Bz; // 状态变量

    // 中间变量
    //  IntermediateState
    // 参考四元数
    //  ReferenceState q_ref_w,  q_ref_x, q_ref_y, q_ref_z;
    Control omega_Bx, omega_By, omega_Bz;
    // Control tau_x, tau_y, tau_z;                    // 控制变量
    Control T; // 控制变量

    int N = 15;
    double dt = 0.01;
    const double m = 1;   // 质量
    const double g = 9.8; // 重力加速度
    const double kdx = 0;
    const double kdy = 0;
    const double kdz = 0;
    const double kh = 0;
    const double kT = 9.8 / 0.593687;

    // 定义微分方程
    DifferentialEquation f;
    f << dot(ksi_x) == ksi_vx;
    f << dot(ksi_y) == ksi_vy;
    f << dot(ksi_z) == ksi_vz;
    f << dot(ksi_vx) == T * kT * (2 * qw * qy + 2 * qx * qz) / m;
    f << dot(ksi_vy) == T * kT * (-2 * qw * qx + 2 * qy * qz) / m;
    f << dot(ksi_vz) == T * kT * (qw * qw - qx * qx - qy * qy + qz * qz) / m - g;
    f << dot(qw) == -0.5 * omega_Bx * qx - 0.5 * omega_By * qy - 0.5 * omega_Bz * qz;
    f << dot(qx) == 0.5 * omega_Bx * qw - 0.5 * omega_By * qz + 0.5 * omega_Bz * qy;
    f << dot(qy) == 0.5 * omega_Bx * qz + 0.5 * omega_By * qw - 0.5 * omega_Bz * qx;
    f << dot(qz) == -0.5 * omega_Bx * qy + 0.5 * omega_By * qx + 0.5 * omega_Bz * qw;

    // f << dot(omega_Bx) == -0.928 * omega_By * omega_Bz + 148.787 * tau_x;
    // f << dot(omega_By) == 0.940 * omega_Bx * omega_Bz + 124.363 * tau_y;
    // f << dot(omega_Bz) == -0.0924 * omega_Bx * omega_By + 70.033 * tau_z;

    // 自定义cost fuction

    // 创建一个最小化目标函数
    Function h, hN;

    h << ksi_x << ksi_y << ksi_z << ksi_vx << ksi_vy << ksi_vz << qw << qx << qy << qz << omega_Bx << omega_By << omega_Bz << T;
    hN << ksi_x << ksi_y << ksi_z << ksi_vx << ksi_vy << ksi_vz << qw << qx << qy << qz;

    // Provide defined weighting matrices:
    DMatrix W = eye<double>(h.getDim()) * 1;
    DMatrix WN = eye<double>(hN.getDim()) * 1;
    DVector r(h.getDim());
    r.setZero();
    DVector rN(hN.getDim()); // End cost reference
    rN.setZero();
    // double coeff_tau = 1;

    double coeff_T = 5.0; // 1

    double coeff_ksi = 200;
    double coeff_ksi_z = 500;
    double coeff_ksi_Nx = 200;
    double coeff_ksi_Nz = 500;
    double coeff_ksi_v = 1;

    double coeff_q_x = 100;
    double coeff_q_y = 100;
    double coeff_q_w = 1;
    double coeff_q_z = 100;

    double coeff_q_N_x = 100;
    double coeff_q_N_y = 100;
    double coeff_q_w_N = 1;
    double coeff_q_z_N = 100;

    double coeff_omega = 0.3; // 0.1

    //     double coeff_T = 1;

    // double coeff_ksi = 200;
    // double coeff_ksi_z = 500;

    // double coeff_ksi_v = 1;

    // double coeff_q = 5;
    // double coeff_q_w = 5;
    // double coeff_q_z = 200;

    // double coeff_omega = 1;

    // double coeff_T = 1;

    W(0, 0) = coeff_ksi;
    W(1, 1) = coeff_ksi;
    W(2, 2) = coeff_ksi_z;

    W(3, 3) = coeff_ksi_v;
    W(4, 4) = coeff_ksi_v;
    W(5, 5) = coeff_ksi_v;

    W(6, 6) = coeff_q_w;
    W(7, 7) = coeff_q_x;
    W(8, 8) = coeff_q_y;
    W(9, 9) = coeff_q_z;

    W(10, 10) = coeff_omega;
    W(11, 11) = coeff_omega;
    W(12, 12) = coeff_omega;

    // W(13, 13) = coeff_tau;
    // W(14, 14) = coeff_tau;
    // W(15, 15) = coeff_tau;

    W(13, 13) = coeff_T;

    WN(0, 0) = coeff_ksi_Nx;
    WN(1, 1) = coeff_ksi_Nx;
    WN(2, 2) = coeff_ksi_Nz;


    double coeff_ksi_v_N = 1;

    WN(3, 3) = coeff_ksi_v_N;
    WN(4, 4) = coeff_ksi_v_N;
    WN(5, 5) = coeff_ksi_v_N;

    WN(6, 6) = coeff_q_w_N;
    WN(7, 7) = coeff_q_N_x;
    WN(8, 8) = coeff_q_N_y;
    WN(9, 9) = coeff_q_z_N;

    // WN(10, 10) = coeff_omega;
    // WN(11, 11) = coeff_omega;
    // WN(12, 12) = coeff_omega;

    // DVector r(h.getDim());
    // r.setAll(5.0);
    // WN *= 50;

    // 创建一个最优化问题
    // 定义最终时间
    double T_end = dt * double(N);
    OCP ocp(0.0, T_end, N);
    if (!CODE_GEN)
    {
        const double x_ref = 0.1; // x position of reference
        const double y_ref = 0.0; // y position of reference
        const double z_ref = 0.1; // z position of reference
        r(0) = x_ref;
        r(1) = y_ref;
        r(2) = z_ref;
        rN(0) = r(0);
        rN(1) = r(1);
        rN(2) = r(2);
        ocp.minimizeLSQ(W, h, r);
        ocp.minimizeLSQEndTerm(WN, hN, rN);
    }
    else
    {
        ocp.minimizeLSQ(W, h);
        ocp.minimizeLSQEndTerm(WN, hN);
    }

    ocp.subjectTo(f);

    ocp.subjectTo(-6.0 <= omega_Bx <= 6.0);
    ocp.subjectTo(-6.0 <= omega_By <= 6.0);
    ocp.subjectTo(-6.0 <= omega_Bz <= 6.0);
    ocp.subjectTo(0.001 <= T <= 1.0);

    if (!CODE_GEN)
    {
        // Set initial state

        ocp.subjectTo(AT_START, ksi_x == 0.0);
        ocp.subjectTo(AT_START, ksi_y == 0.0);
        ocp.subjectTo(AT_START, ksi_z == 0.0);
        ocp.subjectTo(AT_START, ksi_vx == 0.0);
        ocp.subjectTo(AT_START, ksi_vy == 0.0);
        ocp.subjectTo(AT_START, ksi_vz == 0.0);
        ocp.subjectTo(AT_START, qw == 1.0);
        ocp.subjectTo(AT_START, qx == 0.0);
        ocp.subjectTo(AT_START, qy == 0.0);
        ocp.subjectTo(AT_START, qz == 0.0);
        ocp.subjectTo(AT_START, omega_Bx == 0.0);
        ocp.subjectTo(AT_START, omega_By == 0.0);
        ocp.subjectTo(AT_START, omega_Bz == 0.0);
        ocp.subjectTo(AT_START, T == 0.0);
        // Setup some visualization
        GnuplotWindow window1(PLOT_AT_EACH_ITERATION);
        window1.addSubplot(ksi_x, "position x");
        window1.addSubplot(ksi_y, "position y");
        window1.addSubplot(ksi_z, "position z");
        GnuplotWindow window3(PLOT_AT_EACH_ITERATION);
        window3.addSubplot(T, "Thrust");
        window3.addSubplot(omega_Bx, "omega x");
        window3.addSubplot(omega_By, "omega y");
        window3.addSubplot(omega_Bz, "omega z");
        // Define an algorithm to solve it.
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
        algorithm.set(KKT_TOLERANCE, 1e-3);
        algorithm << window1;
        // algorithm << window2;
        algorithm << window3;
        algorithm.solve();
    }
    else
    {
        OCPexport mpc(ocp);

        mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        // mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
        mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // good convergence
        mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); // due to qpOASES
        mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
        mpc.set(NUM_INTEGRATOR_STEPS, N);
        // mpc.set(MAX_NUM_QP_ITERATIONS, 20);
        mpc.set(QP_SOLVER, QP_QPOASES);

        mpc.set(HOTSTART_QP, YES);
        mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
        mpc.set(GENERATE_TEST_FILE, YES);
        mpc.set(GENERATE_MAKE_FILE, YES);
        // mpc.set(GENERATE_MATLAB_INTERFACE, YES);
        // mpc.set(GENERATE_SIMULINK_INTERFACE, YES);
        // 	mpc.set( USE_SINGLE_PRECISION,        YES             );

        if (mpc.exportCode("dynamic_optimize") != SUCCESSFUL_RETURN)
            exit(EXIT_FAILURE);

        mpc.printDimensionsQP();
    }
    // 求解
    // OptimizationAlgorithm algorithm(ocp);

    return 0;
}