#include <iostream>
#include "tools.h"
#define MIN_VALUE 0.0001
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.empty()) {
    cout << "ERROR CalculateRMSE: estimations vector is empty" << endl;
    return rmse;
  }

  if (ground_truth.empty()) {
    cout << "ERROR CalculateRMSE: ground-truth vector is empty" << endl;
    return rmse;
  }

  unsigned long estimation_size = estimations.size();
  if (estimation_size != ground_truth.size()) {
    cout << "ERROR CalculateRMSE: ground-truth vector size must be same as estimations vectors size" << endl;
    return rmse;
  }

  for (unsigned int i = 0; i < estimation_size; ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse = rmse / estimation_size;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  if (x_state.size() != 4) {
    cout << "ERROR CalculateJacobian: The state vector must have size 4." << endl;
    return Hj;
  }
  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);
  double px_c2 = px / c2;
  double py_c2 = py / c2;
  double posx_vely = px * vy;
  double posy_velx = py * vx;

  //compute the Jacobian matrix
  Hj << px_c2, py_c2, 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (posy_velx-posx_vely)/c3, px * (posx_vely-posy_velx)/c3, px_c2, py_c2;

  return Hj;
}
