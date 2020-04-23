#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "wheel_odom_integration_base.h"

#include <ceres/ceres.h>


class whOdomFactor : public ceres::SizedCostFunction<3, 3, 3>
{
  public:
  whOdomFactor( WhOdomIntegrationBase* _whOdombase) : whOdombase(_whOdombase) {}


  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
  {
     Eigen::Vector3d Vi(parameters[0][0], parameters[0][1], parameters[0][2]);
     Eigen::Vector3d Vj(parameters[1][0], parameters[2][1], parameters[3][2]);
     //Eigen::Quaterniond Qi(parameters[2][3],parameters[2][0], parameters[2][1], parameters[2][2]);

     Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);
     residual = whOdombase->evaluate(Vi, Vj);

     if(jacobians[0])
     {
      Eigen::Map<Eigen::Matrix<double, 3, 3>> jacobian_vi(jacobians[0]);
      jacobian_vi = Eigen::Matrix3d::Identity();
     }
     if(jacobians[1])
     {
      Eigen::Map<Eigen::Matrix<double, 3, 3>> jacobian_vj(jacobians[1]);
      jacobian_vj = Eigen::Matrix3d::Identity();
     }

      return true;
  }

  WhOdomIntegrationBase* whOdombase;

};
