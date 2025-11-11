#include <fstream>
#include <iostream>
#include <stdexcept>

#include "DARE.h"

using namespace std;

void setup(void) {
  int n = 2;
  double dt = 1 / 30.;

  int horizon = 10;

  ofstream myfile;
  myfile.open("OUT");

  // SISO
  Eigen::MatrixXd A(n, n);
  Eigen::MatrixXd B(n, 1);

  Eigen::MatrixXd Q(n, n);
  Eigen::MatrixXd R(1, 1);

  // System dynamics (2x2 matrix)
  A << 1., 1., 0, 1;
  B << 0., 1.;  // Control only affects velocity (second state)

  Q.setIdentity();  // State cost matrix (2x2 identity)
  R << 1;           // Control cost matrix (1x1 scalar)

  // Construct the  SYSTEM
  DARE contr(A, B, Q, R, horizon);

  contr.init();

  contr.print();

  // RUN SYTEM WIHT THAT CONTROLLER !
  // const IOFormat fmt(5, DontAlignCols, "\t", " ", "", "", "", "");
  // Eigen::IOFormat fmt(5, 0, ", ", ";\n", "[", "]", "[", "]");
  Eigen::IOFormat fmt(5, 0, ", ", " ", "", "", "", "");

  Eigen::VectorXd x(n);  // State vector
  Eigen::VectorXd u(1);  // Control input

  x << 1, 0;  // Initial state: position=1, velocity=0

  // Write header for data file
  myfile << "# Time Position Velocity" << std::endl;

  for (int i = 0; i < horizon; i++) {
    u = -1 * contr.getK(i) * x;  // Compute optimal control
    x = A * x + B * u;           // Apply control and update state
    // std::cout << "X= "<<std::endl;
    std::cout << "x= " << x.format(fmt) << std::endl;
    myfile << i * dt << " " << x.format(fmt) << std::endl;
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
