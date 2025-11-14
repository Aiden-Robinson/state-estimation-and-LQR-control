#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

#include "DARE.h"

using namespace std;

// Physics variables and functions from fall.cpp
double g, hight, v, dt;  // Make dt global

void fall_init(double x0, double gravity) {
  g = gravity;
  hight = x0;
  v = 0;
}

double falling(double noise_p, double noise_g, double u) {
  hight = hight + dt * v + dt * noise_p;
  v = v - dt * (g + noise_g) + dt * u;
  return (hight);
}

void setup(void) {
  int n = 2;
  dt = 1.0 / 30.;  // Set global dt

  int horizon = 10;

  // Noise generators
  std::default_random_engine generator;
  std::normal_distribution<double> p_noise_h(0.0, 0.01 * 0.01);
  std::normal_distribution<double> p_noise_g(0.0, 0.1 * 0.1);

  ofstream myfile;
  myfile.open("OUT");

  // SISO
  Eigen::MatrixXd A(n, n);
  Eigen::MatrixXd B(n, 1);

  Eigen::MatrixXd Q(n, n);
  Eigen::MatrixXd R(1, 1);

  // System dynamics (2x2 matrix) - corrected to match fall.cpp physics
  A << 1., dt,  // height = height + dt*velocity
      0.,
      1.;  // velocity = velocity - dt*g + dt*u (gravity handled in simulation)
  B << 0., dt;  // Control only affects velocity (second state)

  Q.setIdentity();  // State cost matrix (2x2 identity)
  R << 1;           // Control cost matrix (1x1 scalar)

  // Construct the SYSTEM
  DARE contr(A, B, Q, R, horizon);

  contr.init();

  contr.print();

  // Initialize physics from fall.cpp
  fall_init(100.0, 9.81);  // Start at height=100.0, gravity=9.81

  // RUN SYSTEM WITH THAT CONTROLLER !
  // const IOFormat fmt(5, DontAlignCols, "\t", " ", "", "", "", "");
  // Eigen::IOFormat fmt(5, 0, ", ", ";\n", "[", "]", "[", "]");
  Eigen::IOFormat fmt(5, 0, ", ", " ", "", "", "", "");

  Eigen::VectorXd x(n);  // State vector
  Eigen::VectorXd u(1);  // Control input

  x << hight, v;  // Initial state from physics

  // Write header for data file
  myfile << "# Time Position Velocity Control" << std::endl;

  for (int i = 0; i < horizon; i++) {
    u = -1 * contr.getK(i) * x;  // Compute optimal control
    // x = A * x + B * u;           // Apply control and update state

    // Use physics from fall.cpp instead of x = A*x + B*u
    falling(p_noise_h(generator), p_noise_g(generator),
            u[0]);  // modifying global variables

    // Update state vector with current physics state
    x[0] = hight;
    x[1] = v;

    // Print state and control values
    std::cout << "Step " << i << ": height=" << x[0] << ", velocity=" << x[1]
              << ", control=" << u[0] << std::endl;
    myfile << i * dt << " " << x.format(fmt) << " " << u[0] << std::endl;
  };
  myfile.close();

  // Create gnuplot script
  ofstream gnuplot_script;
  gnuplot_script.open("plot_script.gp");
  gnuplot_script << "set terminal png size 800,600" << std::endl;
  gnuplot_script << "set output 'lqr_control.png'" << std::endl;
  gnuplot_script << "set title 'LQR Controller - State Trajectory'"
                 << std::endl;
  gnuplot_script << "set xlabel 'Time (seconds)'" << std::endl;
  gnuplot_script << "set ylabel 'State Value'" << std::endl;
  gnuplot_script << "set grid" << std::endl;
  gnuplot_script << "plot 'OUT' using 1:2 with lines title 'Position', \\"
                 << std::endl;
  gnuplot_script << "     'OUT' using 1:3 with lines title 'Velocity'"
                 << std::endl;
  gnuplot_script.close();

  // Execute gnuplot
  system("gnuplot plot_script.gp");
  std::cout << "Plot saved as 'lqr_control.png'" << std::endl;
}

int main(void) {
  setup();
  return (0);
}
