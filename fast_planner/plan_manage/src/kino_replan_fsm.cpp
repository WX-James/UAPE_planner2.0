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


#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualizer.reset(new Visualizer(nh));
  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  cmd_timer_ = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::pubCMDCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  target_sub_ = 
      nh.subscribe("/target_box", 1, &KinoReplanFSM::targetCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  traj_pub_    = nh.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20);
  cmd_pub_     = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 20);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  // visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::targetCallback(const obj_state_msgs::ObjectsStatesConstPtr& msg) {
  cout << "Triggered!" << endl;
  trigger_ = true;
  Eigen::Vector3d tar_v_;
  int area;
  for(auto state: msg->states) {
    end_pt_(0) = state.position.x;
    end_pt_(1) = state.position.y;
    end_pt_(2) = state.position.z;
    tar_v_ << state.velocity.x, state.velocity.y, state.velocity.z;
    area = state.area;
  }
  end_vel_.setZero();
  have_target_ = true;

  if(tar_v_.head(2).norm() < 0.3 && area > 20000 && area < 23000) {
    changeFSMExecState(WAIT_TARGET, "FSM");
    return;
  }

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::pubCMDCallback(const ros::TimerEvent& e) {

  MidPlanData* info = &planner_manager_->plan_data_;
  ros::Time      time_now = ros::Time::now();
  double         t_cur    = (time_now - info->start_time_).toSec();
  if(info->traj.getPieceNum() > 0) {
      t_cur = min(info->traj.getTotalDuration(), t_cur);

      double pos_gain[3] = { 7.0, 7.0, 7.5 };
      double vel_gain[3] = { 5.4, 5.4, 5.0 };
      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      Eigen::Vector3d vel = info->traj.getVel(t_cur);
      Eigen::Vector3d acc = info->traj.getAcc(t_cur);
      double desire_psi;
      planner_manager_->planYaw(t_cur, desire_psi);

      cmd_.position.x = pos[0];
      cmd_.position.y = pos[1];
      cmd_.position.z = pos[2];
      cmd_.velocity.x = vel[0];
      cmd_.velocity.y = vel[1];
      cmd_.velocity.z = vel[2];
      cmd_.acceleration.x = acc[0];
      cmd_.acceleration.y = acc[1];
      cmd_.acceleration.z = acc[2];
      cmd_.kx[0] = pos_gain[0];
      cmd_.kx[1] = pos_gain[1];
      cmd_.kx[2] = pos_gain[2];
      cmd_.kv[0] = vel_gain[0];
      cmd_.kv[1] = vel_gain[1];
      cmd_.kv[2] = vel_gain[2];
      cmd_.yaw = desire_psi;
        // publish control command
      cmd_pub_.publish(cmd_);
  }
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0) = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      // if (!success){
      //   have_target_ = false;
      //   changeFSMExecState(WAIT_TARGET, "FSM");
      //   // changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      // }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      MidPlanData* info = &planner_manager_->plan_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur = min(info->traj.getTotalDuration(), t_cur);

      // /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->traj.getTotalDuration() - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      } 

      break;
    }

    case REPLAN_TRAJ: {
      MidPlanData* info = &planner_manager_->plan_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      if(t_cur > info->traj.getTotalDuration()) {
        start_pt_  = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_ << 0, 0, 0;
      } else {
        start_pt_  = info->traj.getPos(t_cur);
        start_vel_ = info->traj.getVel(t_cur);
        start_acc_ = info->traj.getAcc(t_cur);
      }

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  MidPlanData* info = &planner_manager_->plan_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->traj.getTotalDuration()) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);
    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->traj.getTotalDuration()) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "Change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
      }  else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "Goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }

  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("Current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }

}

bool KinoReplanFSM::callKinodynamicReplan() {

  auto plan_data = &planner_manager_->plan_data_;

  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  bool plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
  double compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Kino Path Search Time Cost (ms)： " << compTime <<std::endl;

  if (plan_success) {
    tic = std::chrono::high_resolution_clock::now();

    plan_success = planner_manager_->sfcGen();

    compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    std::cout << "SFC Generation Cost (ms)： " << compTime <<std::endl;

    plan_success = planner_manager_->trajOpt(start_pt_, start_vel_, start_acc_);

    compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    std::cout << "Traj Opt Time Cost (ms)： " << compTime <<std::endl;

    if (plan_success) {
      /* visulization */
      // auto plan_data = &planner_manager_->plan_data_;

      visualizer->visualizePolytope(plan_data->hPolys);   
      if (plan_data->traj.getPieceNum() > 0){
        visualizer->visualize(plan_data->traj, plan_data->kino_path_);
      }

      plan_data->start_time_ = ros::Time::now();

      ros::Duration(0.001).sleep();

      return true;
    } 
    else {
      cout << "generate new traj fail." << endl;
      return false;
    }
  }
  else {
    cout << "path search fail." << endl;
    return false;
  }

}

// KinoReplanFSM::
}  // namespace fast_planner
