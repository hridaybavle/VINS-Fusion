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
  ~whOdomFactor() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
  {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    //Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    //Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Vi, Vj;

    //Eigen::Quaterniond Qi(parameters[2][3],parameters[2][0], parameters[2][1], parameters[2][2]);
    //std::cout << "Vj" << Vj << std::endl;
    //std::cout << "Vi" << Vi << std::endl;
    Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);
    residual = whOdombase->evaluate(Pi, Vi, Pj, Vj);

    if(jacobians)
    {
      if(jacobians[0])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i = Eigen::Matrix3d::Identity();
//        jacobians[0][0] = 1;jacobians[0][1] = 0;jacobians[0][2] = 0;jacobians[0][3] = 0;jacobians[0][4] = 0;jacobians[0][5] = 0;jacobians[0][6] = 0;
//        jacobians[0][7] = 0;jacobians[0][8] = 1;jacobians[0][9] = 0;jacobians[0][10] = 0;jacobians[0][11] = 0;jacobians[0][12] = 0;jacobians[0][13] = 0;
//        jacobians[0][14] = 0;jacobians[0][15] = 0;jacobians[0][16] = 1;jacobians[0][17] = 0;jacobians[0][18] = 0;jacobians[0][19] = 0;jacobians[0][20] = 0;
      }
      if(jacobians[1])
      {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
        jacobian_pose_j = Eigen::Matrix3d::Identity();
//        jacobians[1][0] = 1;jacobians[1][1] = 0;jacobians[1][2] = 0;jacobians[1][3] = 0;jacobians[1][4] = 0;jacobians[1][5] = 0;jacobians[1][6] = 0;
//        jacobians[1][7] = 0;jacobians[1][8] = 1;jacobians[1][9] = 0;jacobians[1][10] = 0;jacobians[1][11] = 0;jacobians[1][12] = 0;jacobians[0][13] = 0;
//        jacobians[1][14] = 0;jacobians[1][15] = 0;jacobians[1][16] = 1;jacobians[1][17] = 0;jacobians[1][18] = 0;jacobians[1][19] = 0;jacobians[1][20] = 0;
      }

    }

    return true;
  }

  WhOdomIntegrationBase* whOdombase;

};
