/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include <gcopter/trajectory.hpp>
#include <path_searching/topo_prm.h>

using std::vector;

namespace fast_planner {

struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double local_traj_len_;                // local replanning trajectory length
  double ctrl_pt_dist;                   // distance between adjacient B-spline
                                         // control points
  double clearance_;
  int dynamic_;
  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;
};

struct LocalTrajData {
  /* info of generated traj */

  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_;
  
};

class MidPlanData {
public:
  MidPlanData(/* args */) {}
  ~MidPlanData() {}

  // kinodynamic path
  vector<Eigen::Vector3d> kino_path_;
  // sfc
  vector<Eigen::MatrixX4d> hPolys;
  // traj;
  ros::Time start_time_;
  Trajectory<5> traj;

  // heading planning

  double last_yaw=0;

};

}  // namespace fast_planner

#endif