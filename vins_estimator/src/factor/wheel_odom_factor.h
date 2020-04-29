#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "wheel_odom_integration_base.h"

#include <ceres/ceres.h>

class VelCostFunctor {

public:
  VelCostFunctor(WhOdomIntegrationBase* _pre_integration): pre_integration(_pre_integration)
  {

  }

  bool operator()(const double* const x,
                  const double* const y,
                  double* residuals) const {

    Eigen::Vector3d Pi, Pj;

    Pi << x[0], x[1], x[2];
    Pj << y[0], y[1], y[2];

    std::cout << "Pi: " << Pi << std::endl;
    std::cout << "Pi: " << Pj << std::endl;

    Eigen::Map<Eigen::Matrix<double, 3, 1>> comp_residuals(residuals);
    comp_residuals = pre_integration->evaluate(Pi, Pj);

    //    residuals[0] = y[0] - x[0];
    //    residuals[1] = y[1] - x[1];
    //    residuals[2] = y[2] - x[2];



    //std::cout << "residuals:" << residuals[0] << std::endl << residuals[1] << std::endl << residuals[2] << std::endl;
    return true;
  }

  WhOdomIntegrationBase* pre_integration;

};
