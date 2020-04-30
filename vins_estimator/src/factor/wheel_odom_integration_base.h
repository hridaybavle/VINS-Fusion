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
    WhOdomIntegrationBase() = delete;
    WhOdomIntegrationBase(const Eigen::Vector3d &_vel_0) : vel_0{_vel_0}, delta_p{Eigen::Vector3d::Zero()}, delta_v{Eigen::Vector3d::Zero()}, sum_dt{0}
    {

    }

    void push_back(double dt, const Eigen::Vector3d vel)
    {
      vel_buf.push_back(vel);
      propagate(dt, vel);
    }


    void repropagate(const Eigen::Vector3d vel)
    {
      delta_v.setZero();
    }

    void propagate(double _dt, const Eigen::Vector3d& _vel_1)
    {
      dt = _dt;
      vel_1 = _vel_1;
      midpointIntegration(_dt, vel_0, _vel_1);
    }

    void midpointIntegration(double _dt,
                             Eigen::Vector3d _vel_0, Eigen::Vector3d _vel_1)
    {
      dt = _dt;
      Vector3d un_vel_0 = _vel_0;
      Vector3d un_vel_1 = _vel_1;
      Vector3d un_vel   = 0.5 * (vel_0 + vel_1);

      //delta_p = delta_p + un_vel * dt;
      delta_v = un_vel;
      sum_dt += dt;
      vel_0   = _vel_1;
    }

    Eigen::Matrix<double, 3, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Vector3d &Pj)
    {
      Eigen::Matrix<double, 3, 1> residuals;
      residuals.setZero();

      if(sum_dt == 0){
        std::cout << "returning residuals as dt is 0" << std::endl;
        return residuals;
      }

      Eigen::Vector3d pred_v = (Pj - Pi);
      //std::cout << "pred v: " << pred_v << std::endl;
      //std::cout << "delt_v: " << delta_v << std::endl;
      residuals.block<3,1>(0,0) = pred_v - delta_v*sum_dt;
      //std::cout << "sum_dt: " << sum_dt << std::endl;
      //std::cout << "vel residual:" << residuals << std::endl;

      //vel_buf.clear();
      return residuals;
    }

    double dt;
    double sum_dt;
    std::vector<Eigen::Vector3d> vel_buf;
    Eigen::Vector3d meas_avg_vel;
    Eigen::Vector3d delta_p, delta_v;
    Eigen::Vector3d vel_0, vel_1;
};
