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

    //std::cout << "Pi: " << Pi << std::endl;
    //std::cout << "Pj: " << Pj << std::endl;

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

class PoseDiffCostFunctor {

public:
  PoseDiffCostFunctor()
  {

  }

  bool operator()(const double* const x,
                  const double* const y,
                  double* residuals) const {

    std::cout << "Pw in optimization: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
    std::cout << "Ps in optimization: " << y[0] << ", " << y[1] << ", " << y[2] << std::endl;

    residuals[0] = (y[0] - x[0]);
    residuals[1] = (y[1] - x[1]);
    residuals[2] = (y[2] - x[2]);

    //std::cout << "pose residuals: " << residuals[0] << std::endl << residuals[1] << std::endl << residuals[2] << std::endl;

    return true;
  }

};

class AutoDiffPoseCostFunctor {

public:
  AutoDiffPoseCostFunctor(WhOdomIntegrationBase* _pre_integration): pre_integration(_pre_integration)
  {

  }

  template <typename T>
  bool operator()(const T* const x,
                  const T* const y,
                  T* residuals) const {

    Eigen::Vector3d meas_p;
    meas_p = pre_integration->getMeasP();

    double var = 0.9;
    residuals[0] = ((x[0] -  y[0]) - T(meas_p(0)))/ var;
    residuals[1] = ((x[1] -  y[1]) - T(meas_p(1)))/ var;
    residuals[2] = ((x[2] -  y[2]) - T(meas_p(2)))/ var;

    //std::cout << "meas P: " << meas_p << std::endl;
    //std::cout << "pose vel residuals: " << residuals[0] << std::endl << residuals[1] << std::endl << residuals[2] << std::endl;

    return true;
  }

  WhOdomIntegrationBase* pre_integration;

};

class AutoDiffVelCostFunctor {

public:
  AutoDiffVelCostFunctor()
  {

  }

  template <typename T>
  bool operator()(const T* const x,
                  const T* const y,
                  T* residuals) const {

    double var = 0.09;
    residuals[0] = (x[0] -  y[0])/ var;
    residuals[1] = (x[1] -  y[1])/ var;
    residuals[2] = (x[2] -  y[2])/ var;

    //std::cout << "estimated v: " << x[0] << std::endl << x[1] << std::endl << x[2] << std::endl;
    //std::cout << "meas v: " << y[0] << std::endl << y[1] << std::endl << y[2] << std::endl;
    //std::cout << "vel residuals: " << residuals[0] << std::endl << residuals[1] << std::endl << residuals[2] << std::endl;

    return true;
  }

};
