#include <Eigen/Dense>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

#include "DARE.h"
#include "kalman.hpp"

using namespace std;

#define P_SIGMA_H 0.01  // position noise
#define P_SIGMA_G 0.1   // gravity noise

#define M_SIGMA 1.

double g, hight, v, t;

double dt;

void fall_init(double x0, double gravity) {
  g = gravity;
  hight = x0;
  v = 0;
}

// falling with noise injected and control input u.
double falling(double noise_p, double noise_g, double u) {
  hight = hight + dt * v + dt * noise_p;
  v = v - dt * (g + noise_g) + dt * u;

  return (hight);
}

int main(void) {
  // Kalman filter setup
  int n = 3;  // Number of states
  int m = 1;  // Number of measurements

  // SOME NOISE
  std::default_random_engine generator;
  std::normal_distribution<double> p_noise_h(0.0, P_SIGMA_H * P_SIGMA_H);
  std::normal_distribution<double> p_noise_g(0.0, P_SIGMA_G * P_SIGMA_G);
  std::normal_distribution<double> m_noise(0.0, M_SIGMA * M_SIGMA);

  dt = 1.0 / 30;  // Time step

  Eigen::MatrixXd A(n, n);  // System dynamics matrix
  Eigen::MatrixXd C(m, n);  // Output matrix
  Eigen::MatrixXd Q(n, n);  // Process noise covariance
  Eigen::MatrixXd R(m, m);  // Measurement noise covariance
  Eigen::MatrixXd P(n, n);  // Estimate error covariance

  A << 1., dt, 0, 0, 1, dt, 0, 0,
      1;  // the last row is 0 0 1 cause gravity is treated like a constant

  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << P_SIGMA_H * P_SIGMA_H, 0., 0, 0, 0, 0, 0, 0, P_SIGMA_G * P_SIGMA_G;
  R << M_SIGMA * M_SIGMA;

  P << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  // initilze the kalman filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // initial guess at pos
  Eigen::VectorXd x0(n);
  // The third is initial guess of gravity
  x0 << 110, 0, 0;
  // x0 << 100,0,2; ///  WOrks even with wrong sign
  kf.init(dt, x0);

  // true values of the system
  fall_init(100, 5.5);  // Physical conditions

  FILE* log;
  log = fopen("Data", "w");

  double t = 0;

  Eigen::VectorXd y(m);

  y << falling(0., 0., 0.);  // initialize estimated

  fprintf(log, "%f\t %f\t%f\t%f\t%f\t %f\t%f\n", t, y(0), kf.state()[0],
          kf.state()[1], kf.state()[2], hight, v);
  // prints the time, measured height, est pos, est velocity, and est gravity

  // LQR definitions

  // SISO
  Eigen::MatrixXd A_lqr(n, n);
  Eigen::MatrixXd B_lqr(n, 1);

  Eigen::MatrixXd Q_lqr(n, n);
  Eigen::MatrixXd R_lqr(1, 1);

  // System dynamics (3x3 matrix) - corrected to match fall.cpp physics
  A_lqr << 1., dt, 0,  // height = height + dt*velocity + 0*gravity
      0., 1, -dt,      // velocity = velocity - dt*gravity + dt*u
      0., 0, 1;        // gravity = gravity (constant)

  B_lqr << 0,  // position not directly affected by control
      dt,      // velocity affected by dt*u
      0;       // gravity not affected by control

  Q_lqr.setIdentity();  // State cost matrix (2x2 identity)
  R_lqr << 1;           // Control cost matrix (1x1 scalar)

  int horizon = 10;
  DARE contr(A_lqr, B_lqr, Q_lqr, R_lqr, horizon);

  contr.init();

  while (y[0] > 0.) {
    t += dt;

    kf.update(y);
    fprintf(log, "%f\t %f\t%f\t%f\t%f\t %f\t%f\n", t, y(0), kf.state()[0],
            kf.state()[1], kf.state()[2], hight, v);
    // prints the time, measured height, est pos, est velocity, and est
    // gravity
    y << falling(p_noise_h(generator), p_noise_g(generator), 0.);
    // ADD COME NOISE
    y(0) += m_noise(generator);
  }
  fflush(log);
  fclose(log);
  // printing with gnuplot
  string cmd =
      "gnuplot -persist -e \""
      "set title 'Fall + Kalman (height)'; "
      "set xlabel 'time [s]'; "
      "set ylabel 'height [m]'; "
      "plot "
      "'Data' using 1:2 with points pointtype 7 pointsize 0.5 title "
      "'measured "
      "h', "
      "'Data' using 1:3 with lines lw 2 title 'estimated h', "
      "'Data' using 1:6 with lines lw 2 dashtype 2 title 'true h',"
      "'Data' using 1:4 with lines lw 2 dashtype 3 title 'est vel', "
      "'Data' using 1:5 with lines lw 2 title 'estimated g'\"";

  system(cmd.c_str());

  return (0);
}

// g++ -I /usr/include/eigen3 -I . main.cpp DARE.cpp kalman.cpp -o lander