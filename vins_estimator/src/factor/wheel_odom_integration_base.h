/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include "../utility/utility.h"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

class WhOdomIntegrationBase
{
  public:
    //WhOdomIntegrationBase() = delete;
    WhOdomIntegrationBase()
    {

    }

    void push_back(double dt, const Eigen::Vector3d vel)
    {
      vel_buf.push_back(vel);
    }


    void repropagate(const Eigen::Vector3d vel)
    {

    }


    Eigen::Matrix<double, 3, 1> evaluate(const Eigen::Vector3d &Vi, const Eigen::Vector3d &Vj)
    {
      Eigen::Matrix<double, 3, 1> residuals;

      if(vel_buf.size() == 0)
        return residuals;

      std::cout << "entering eval" << std::endl;

      for(size_t i = 0; i < vel_buf.size(); ++i)
      {
        meas_avg_vel = meas_avg_vel + vel_buf[0];
      }
      meas_avg_vel = meas_avg_vel / vel_buf.size();
      residuals.block<3,1>(0,0) = /*Qi.inverse() * */ (Vj - Vi) - meas_avg_vel;

      vel_buf.clear();
      std::cout << "exiting eval" << std::endl;

      return residuals;
    }

    std::vector<Eigen::Vector3d> vel_buf;
    Eigen::Vector3d meas_avg_vel;
};
