#include <memory>
#include <math.h>
#include <cmath>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <Yaml.hpp>

#define RED "\033[31m"
#define YELLOW "\033[33m"
#define RESET "\033[0m"

/*
 * Ordinary quadrotor NMPC with external force compensation.
 *
 * State:
 *   x = [p_x,p_y,p_z, q_w,q_x,q_y,q_z, v_x,v_y,v_z]
 * Input:
 *   u = [T, w_x,w_y,w_z]
 * OnlineData:
 *   [mass_q, f_q_x,f_q_y,f_q_z]
 *
 * f_Q 是世界系等效外力，单位 N；T 是物理总推力，单位 N，
 * 不是 MAVROS/PX4 的归一化 thrust。
 */
#define CODE_GEN true

int main(int argc, char **argv)
{
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

  std::cout << "Parameters file path: " << cfg_file_path << RESET << std::endl
            << std::endl;

  DifferentialState p_x, p_y, p_z;
  DifferentialState q_w, q_x, q_y, q_z;
  DifferentialState v_x, v_y, v_z;

  Control T, w_x, w_y, w_z;
  DifferentialEquation f;
  Function h, hN;

#if CODE_GEN
  OnlineData Mq;                  // Quadrotor mass [kg]
  OnlineData fq_x, fq_y, fq_z;    // 世界系无人机外力 [N]
#else
  double Mq = cfg_root["mass_q"].As<double>(0.98);
  double fq_x = 0.0, fq_y = 0.0, fq_z = 0.0;
#endif

  double g_z = cfg_root["gravity"].As<double>(9.81);
  double dt = cfg_root["step_T"].As<double>(0.05);
  int N = cfg_root["step_N"].As<int>(20);
  // NMPC 世界系速度硬约束，单位 m/s；x/y 使用同一上限，z 单独限制。
  const double max_velocity_xy = cfg_root["max_velocity_xy"].As<double>(0.3);
  const double max_velocity_z = cfg_root["max_velocity_z"].As<double>(0.3);
  if (!std::isfinite(max_velocity_xy) || !std::isfinite(max_velocity_z) ||
      max_velocity_xy <= 0.0 || max_velocity_z <= 0.0)
  {
    std::cerr << RED << "max_velocity_xy/max_velocity_z must be finite and positive."
              << RESET << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << YELLOW;
  std::cout << "Mq:       " << Mq << std::endl;
  std::cout << "g_z:      " << g_z << std::endl;
  std::cout << "N:        " << N << std::endl;
  std::cout << "dt:       " << dt << std::endl;
  std::cout << "fq_x:     " << fq_x << std::endl;
  std::cout << "fq_y:     " << fq_y << std::endl;
  std::cout << "fq_z:     " << fq_z << std::endl;
  std::cout << RESET << std::endl;

  const double w_max_yaw = 1.0;
  const double w_max_xy = 5.0;
  const double T_min = 4.0;
  const double T_max = 40.0;

  IntermediateState q_sqr;
  IntermediateState b3_x, b3_y, b3_z;
  q_sqr = q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z;
  b3_x = 2.0 * (q_w*q_y + q_x*q_z);
  b3_y = 2.0 * (q_y*q_z - q_w*q_x);
  b3_z = q_sqr;

  f << dot(p_x) == v_x;
  f << dot(p_y) == v_y;
  f << dot(p_z) == v_z;

  f << dot(q_w) == 0.5 * (-w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) == 0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) == 0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) == 0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);

  f << dot(v_x) == (T * b3_x + fq_x) / Mq;
  f << dot(v_y) == (T * b3_y + fq_y) / Mq;
  f << dot(v_z) == -g_z + (T * b3_z + fq_z) / Mq;

  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << T << w_x << w_y << w_z;

  hN << p_x << p_y << p_z
     << q_w << q_x << q_y << q_z
     << v_x << v_y << v_z;

  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  DVector r(h.getDim());
  r.setZero();
  r(3) = 1.0;

  DVector rN(hN.getDim());
  rN.setZero();
  rN(3) = 1.0;

  OCP ocp(0.0, dt * N, N);
#if (!CODE_GEN)
  {
    Q(0, 0) = 100;
    Q(1, 1) = 100;
    Q(2, 2) = 100;
    Q(3, 3) = 100;
    Q(4, 4) = 100;
    Q(5, 5) = 100;
    Q(6, 6) = 100;
    Q(7, 7) = 10;
    Q(8, 8) = 10;
    Q(9, 9) = 10;
    Q(10, 10) = 1;
    Q(11, 11) = 1;
    Q(12, 12) = 1;
    Q(13, 13) = 1;

    DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    QN(0, 0) = Q(0, 0);
    QN(1, 1) = Q(1, 1);
    QN(2, 2) = Q(2, 2);
    QN(3, 3) = Q(3, 3);
    QN(4, 4) = Q(4, 4);
    QN(5, 5) = Q(5, 5);
    QN(6, 6) = Q(6, 6);
    QN(7, 7) = Q(7, 7);
    QN(8, 8) = Q(8, 8);
    QN(9, 9) = Q(9, 9);

    r(0) = 0.5;
    r(2) = 0.6;
    r(10) = g_z * Mq;
    rN(0) = r(0);
    rN(2) = r(2);
    rN(3) = r(3);
    ocp.minimizeLSQ(Q, h, r);
    ocp.minimizeLSQEndTerm(QN, hN, rN);
  }
#else
  {
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN);
  }
#endif

  ocp.subjectTo(f);
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo(T_min <= T <= T_max);
  // v_x/v_y/v_z 是 ENU 世界系速度，单位 m/s；这是求解器的硬约束。
  ocp.subjectTo(-max_velocity_xy, v_x, max_velocity_xy);
  ocp.subjectTo(-max_velocity_xy, v_y, max_velocity_xy);
  ocp.subjectTo(-max_velocity_z, v_z, max_velocity_z);

  ocp.setNOD(4);

#if (!CODE_GEN)
  {
    ocp.subjectTo(AT_START, p_x == 0.0);
    ocp.subjectTo(AT_START, p_y == 0.0);
    ocp.subjectTo(AT_START, p_z == 0.6);
    ocp.subjectTo(AT_START, q_w == 1.0);
    ocp.subjectTo(AT_START, q_x == 0.0);
    ocp.subjectTo(AT_START, q_y == 0.0);
    ocp.subjectTo(AT_START, q_z == 0.0);
    ocp.subjectTo(AT_START, v_x == 0.0);
    ocp.subjectTo(AT_START, v_y == 0.0);
    ocp.subjectTo(AT_START, v_z == 0.0);

    GnuplotWindow window1(PLOT_AT_EACH_ITERATION);
    window1.addSubplot(p_x, "position x");
    window1.addSubplot(p_y, "position y");
    window1.addSubplot(p_z, "position z");

    GnuplotWindow window3(PLOT_AT_EACH_ITERATION);
    window3.addSubplot(T, "Thrust");
    window3.addSubplot(w_x, "body rate x");
    window3.addSubplot(w_y, "body rate y");
    window3.addSubplot(w_z, "body rate z");

    OptimizationAlgorithm algorithm(ocp);
    algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
    algorithm.set(KKT_TOLERANCE, 1e-3);
    algorithm << window1;
    algorithm << window3;
    algorithm.solve();
  }
#else
  {
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
    mpc.set(NUM_INTEGRATOR_STEPS, N);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);
    mpc.set(LEVENBERG_MARQUARDT, 1e-10);
    mpc.set(CG_USE_OPENMP, YES);
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
    mpc.set(USE_SINGLE_PRECISION, YES);

    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    if (mpc.exportCode("quadrotor_payload_mpc") != SUCCESSFUL_RETURN)
      exit(EXIT_FAILURE);
    mpc.printDimensionsQP();
  }
#endif

  return EXIT_SUCCESS;
}
