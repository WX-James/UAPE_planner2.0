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



#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/plan_container.hpp>
#include <gcopter/sfc_gen.hpp>
#include <gcopter/gcopter.hpp>
#include <gcopter/trajectory.hpp>
#include <chrono>
#include <ros/ros.h>

namespace fast_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
  
  bool sfcGen();

  bool trajOpt();

  void planYaw(const double& t_cur, double& desire_psi_);

  void initPlanModules(ros::NodeHandle& nh);

  bool checkTrajCollision(double& distance);

  PlanParameters pp_;
  LocalTrajData local_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;

private:
  /* main planning algorithms & modules */
  SDFMap::Ptr sdf_map_;
  gcopter::GCOPTER_PolytopeSFC gcopter;

  unique_ptr<Astar> geo_path_finder_;
  unique_ptr<KinodynamicAstar> kino_path_finder_;
  

  // heading planning

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION
};
}  // namespace fast_planner

#endif