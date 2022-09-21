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



// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

}


bool FastPlannerManager::checkTrajCollision(double& distance) {

  ros::Time time_now = ros::Time::now();
  double t_cur    = (time_now - plan_data_.start_time_).toSec();
  Eigen::Vector3d cur_pt = plan_data_.traj.getPos(t_cur);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_cur + fut_t < plan_data_.traj.getTotalDuration()) {
    fut_pt = plan_data_.traj.getPos(t_cur + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  local_data_.start_time_ = ros::Time::now();
  plan_data_.start_time_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.2);
  t_search = (ros::Time::now() - t1).toSec();

  std::vector<Eigen::Vector3d> pc;
  sdf_map_->getSurf(pc);

  sfc_gen::convexCover(plan_data_.kino_path_,
                       pc,
                       sdf_map_->getOrigin(),
                       sdf_map_->getCorner(),
                       2.0,
                       3.0,
                       plan_data_.hPolys);
  // sfc_gen::shortCut(plan_data_.hPolys);

  return true;
}

// !SECTION

bool FastPlannerManager::trajOpt()
{
  // Trajectory<5> traj;
  Eigen::Matrix3d iniState;
  Eigen::Matrix3d finState;  
  iniState << plan_data_.kino_path_.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finState << plan_data_.kino_path_.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  Eigen::VectorXd magnitudeBounds(5);
  Eigen::VectorXd penaltyWeights(5);
  Eigen::VectorXd physicalParams(6);
  magnitudeBounds(0) = 2.5;
  magnitudeBounds(1) = 2.1;
  magnitudeBounds(2) = 1.05;
  magnitudeBounds(3) = 2.0;
  magnitudeBounds(4) = 12.0;
  penaltyWeights(0) = 1.0e+4;
  penaltyWeights(1) = 1.0e+4;
  penaltyWeights(2) = 1.0e+4;
  penaltyWeights(3) = 1.0e+4;
  penaltyWeights(4) = 1.0e+5;
  physicalParams(0) = 0.61;
  physicalParams(1) = 9.8;
  physicalParams(2) = 0.7;
  physicalParams(3) = 0.8;
  physicalParams(4) = 0.01;
  physicalParams(5) = 0.0001;
  const int quadratureRes = 15;
  plan_data_.traj.clear();
  if (!gcopter.setup(20,
                     iniState, finState,
                     plan_data_.hPolys, INFINITY,
                     1.0e-2,
                     quadratureRes,
                     magnitudeBounds,
                     penaltyWeights,
                     physicalParams))
  {
    return false;
  }
  if (std::isinf(gcopter.optimize(plan_data_.traj, 1.0e-5)))
  {
    return true;
  }
  
}

// !SECTION

void FastPlannerManager::planYaw(const double& t_cur,
                                 double& desire_psi_) {

  Eigen::Vector2d v2;
  Eigen::Vector2d v1;
  double t_total = plan_data_.traj.getTotalDuration();
  v2 << (plan_data_.traj.getPos(t_total) - plan_data_.traj.getPos(t_cur)).head(2);
  // v2 << end_pt_[0] - odom_pos_[0], end_pt_[1] - odom_pos_[1];
  v1 << 1.0,0.0;

  desire_psi_ = acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
  if (v2(1)<0)
    desire_psi_ = -desire_psi_;

  if (v2.norm()<0.5)
    desire_psi_ = plan_data_.last_yaw;

  // if (desire_psi_ - plan_data_.last_yaw > 1.0)
  //   desire_psi_ = plan_data_.last_yaw + 1.0;
  // if (desire_psi_ - plan_data_.last_yaw < -1.0)
  //   desire_psi_ = plan_data_.last_yaw - 1.0;

  plan_data_.last_yaw = desire_psi_;

}


}  // namespace fast_planner
