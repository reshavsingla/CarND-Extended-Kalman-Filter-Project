#include <iostream>
#include "tools.h"

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
    rmse << 0,0,0,0;

    if(estimations.size() != ground_truth.size() || estimations.empty()){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    for(int i = 0;i < estimations.size();++i)
    {
        VectorXd diff = estimations[i] - ground_truth[i];
        rmse = diff.array().square();
    }

    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

    double px = x_state[0];
    double py = x_state[1];
    double vx = x_state[2];
    double vy = x_state[3];

    double c1 = pow(px,2) + pow(py,2);
    double c2 = pow(c1,0.5);

    MatrixXd j_ = MatrixXd(3,4);
    if(c1 != 0)
    {
        j_<< px/c2,py/c2,0,0,
                -py/c1,px/c1,0,0,
                py*(vx*py-vy*px)/(c1*c2),px*(vy*px-vx*py)/(c1*c2),px/c2,py/c2;
    }
    return j_;
}
