#include <memory>
#include <math.h>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <Yaml.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input.
#define RED     "\033[31m"    /* Red */
#define YELLOW  "\033[33m"    /* Yellow */
#define RESET   "\033[0m"


/*
Switch between code generation and analysis.

If CODE_GEN is true the system is compiled into an optimizaiton problem
for real-time iteration and all code to run it online is generated.
Constraints and reference structure is used but the values will be set on
runtinme.

If CODE_GEN is false, the system is compiled into a standalone optimization
and solved on execution. The reference and constraints must be set in here.
*/

// if you want use your own parameters, run the program with your own parameter's file path. Such as,
//  ./quadrotor_payload_mpc_codegen ../config/mpc.yaml
int main(int argc, char **argv)
{
  // Use Acado
  USING_NAMESPACE_ACADO

  std::string cfg_file_path = "../config/model.yaml";
  if (argc > 1)
    cfg_file_path = argv[1];
  Yaml::Node cfg_root;

  try
  {
    Yaml::Parse(cfg_root, cfg_file_path.data());
  }
  catch (const std::exception &e)
  {
    std::cerr << RED << e.what() << std::endl;
  }
  std::cout << "Parameters file path: " << cfg_file_path << RESET << std::endl;

  // System variables
  DifferentialState p_x, p_y, p_z;
  DifferentialState q_w, q_x, q_y, q_z;
  DifferentialState v_x, v_y, v_z;

  // Control input
  Control a_z, w_x, w_y, w_z;
  DifferentialEquation f; // 微分方程
  Function h, hN; // h-internal step; hN-terminal step
  
  // dynamics
  double g = cfg_root["gravity"].As<double>(9.81);         // Gravity is everywhere [m/s^2]
  double dt = cfg_root["step_T"].As<double>(0.05);         // Discretization time [s]
  int N = cfg_root["step_N"].As<int>(20);                  // Number of nodes
  int delay_N = cfg_root["step_Delay"].As<int>(1);         // Number of nodes for delay

  // std::cout << YELLOW;
  // std::cout << "g:      " << g << std::endl;
  // std::cout << "N:        " << N << std::endl;
  // std::cout << "dt:       " << dt << std::endl;

  // Constraints
  const double w_max_yaw = 1.2;                       // Maximal yaw rate [rad/s]
  const double w_max_xy = 3.0;                        // Maximal pitch and roll rate [rad/s]
  const double a_max_z = 20.0;                       // Maximal acceleration_z [m/s2]
  const double v_max = 10.0;                      // Maximal velocity [m/s]
 
  // Differemtial equation
  f << dot(p_x) == v_x;
  f << dot(p_y) == v_y;
  f << dot(p_z) == v_z;

  f << dot(q_w) == 0.5 * (-w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) == 0.5 * (w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) == 0.5 * (w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) == 0.5 * (w_z * q_w + w_y * q_x - w_x * q_y);

  // NOTE: v在world系下，a在body系下
  f << dot(v_x) == 2 * ( q_w * q_y + q_x * q_z ) * a_z; 
  f << dot(v_y) == 2 * ( q_y * q_z - q_w * q_x ) * a_z;
  // f << dot(v_z) == (1 - 2 * q_x * q_x - 2 * q_y * q_y) * a_z - g;
  f << dot(v_z) == (q_w * q_w + q_z * q_z - q_x * q_x - q_y * q_y) * a_z - g;
  
  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z            // 3
    << q_w << q_x << q_y << q_z     // 7 
    << v_x << v_y << v_z            // 10
    << a_z << w_x << w_y << w_z;    // 14

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
     << q_w << q_x << q_y << q_z
     << v_x << v_y << v_z;

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp(0.0, dt * N , N);

  // For code generation, references are set during run time.
  BMatrix Q_sparse(h.getDim(), h.getDim());
  Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim());
  QN_sparse.setIdentity();
  ocp.minimizeLSQ( Q_sparse, h);
  ocp.minimizeLSQEndTerm( QN_sparse, hN );

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo( 0.0 <= a_z <= a_max_z);
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  // NOTE: 给速度加个约束
  // ocp.subjectTo(-v_max <= v_x <= v_max);
  // ocp.subjectTo(-v_max <= v_y <= v_max);
  // ocp.subjectTo(-v_max <= v_z <= v_max);
  
  // Set online data size 
  // https://github.com/acado/acado/issues/196
  ocp.setNOD(0); // num of OnlineData

  // set online data size
  // For code generation, we can set some properties.
  // The main reason for a setting is given as comment.
  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);    // is robust, stable
  mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING); // good convergence
  mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2); // due to qpOASES
  mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4); // accurate  INT_IRK_GL4
  mpc.set(NUM_INTEGRATOR_STEPS,   N);
  mpc.set(QP_SOLVER,              QP_QPOASES); // free, source code
  mpc.set(HOTSTART_QP,            YES);
  mpc.set(LEVENBERG_MARQUARDT,   1e-10);
  // mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
  // mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
  mpc.set(CG_USE_OPENMP,                      YES); // paralellization
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,      NO);  // set on runtime
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX,   YES); // time-varying costs
  mpc.set(USE_SINGLE_PRECISION,               YES); // Single precision

  // Do not generate tests, makes or matlab-related interfaces.
  mpc.set(GENERATE_TEST_FILE,          NO);
  mpc.set(GENERATE_MAKE_FILE,          NO);
  mpc.set(GENERATE_MATLAB_INTERFACE,   NO);
  mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

  // Finally, export everything.
  if (mpc.exportCode("normal_mpc") != SUCCESSFUL_RETURN)
    exit(EXIT_FAILURE);
    
  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}