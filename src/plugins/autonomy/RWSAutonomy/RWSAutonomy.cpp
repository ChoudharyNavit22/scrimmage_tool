/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <new-scrimmage-plugin/plugins/autonomy/RWSAutonomy/RWSAutonomy.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;
using namespace scrimmage::weapons;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::RWSAutonomy,
                RWSAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

void RWSAutonomy::init(std::map<std::string, std::string> &params) {
  this->pub_fire_bullet_ = advertise("GlobalNetwork", "FireBullet");
  this->pub_turret_aimpoint_ = advertise("LocalNetwork", "AimTurret");

  weapon_range_ = scrimmage::get<double>("weapon_range", params, 100.0);
  fire_rate_ = scrimmage::get<double>("fire_rate", params, 1.0);
  azi_fire_env_ = scrimmage::get<double>("azi_fire_env", params, 0.1);
  elev_fire_env_ = scrimmage::get<double>("elev_fire_env", params, 0.1);
  // pk_ = scrimmage::get<double>("pk", params, 1.0);
  bullet_speed_ = scrimmage::get<double>("bullet_speed", params, 500.0);

  if(fire_rate_ >= 0.01){
    fire_period_ = 1/fire_rate_;
  } else {
    fire_period_ = std::numeric_limits<double>::max();
  }
}

std::tuple<double, int, StatePtr> RWSAutonomy::getTarget(){
  StatePtr pTargetState;
  double min_dist = std::numeric_limits<double>::infinity();
  int target_id = -1;

  // TODO check if alive??
  for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
      // Skip if this contact is on the same team
      if (it->second.id().team_id() == parent_->id().team_id()) {
          continue;
      }

      // Calculate distance to entity
      double dist = (it->second.state()->pos() - state_->pos()).norm();

      if (dist < min_dist) {
          // If this is the minimum distance, save distance and reference to
          // entity
          min_dist = dist;
          target_id = it->first;
          pTargetState = it->second.state();
      }
  }
  return std::make_tuple(min_dist, target_id, pTargetState);
}

std::tuple<double, double, double, double, Eigen::Vector3d> RWSAutonomy::getAim(double dist, Eigen::Vector3d pos, Eigen::Vector3d vel, double dt){
    // Predict target next state
    Eigen::Vector3d est_pos = pos + vel*dt;
    Eigen::Vector3d v_curr = (pos - state_->pos()).normalized();
    Eigen::Vector3d v_pred = (est_pos - state_->pos()).normalized();

    // Convert to spherical coordinates:
    double target_azi = atan2(v_curr(1), v_curr(0));
    double target_elev = atan2(v_curr(2), v_curr.head<2>().norm());
    double target_azi_pred = atan2(v_pred(1), v_pred(0));
    double target_elev_pred = atan2(v_pred(2), v_pred.head<2>().norm());

    double turret_azi = parent_->state()->quat().yaw();
    double turret_elev = -1*parent_->state()->quat().pitch();

    double azi_error = target_azi - turret_azi;
    double elev_error = target_elev - turret_elev;

    if (azi_error < -M_PI){
      azi_error += 2 * M_PI;
    } else if (azi_error > M_PI){
      azi_error -= 2 * M_PI;
    }
    if (elev_error < -M_PI){
      elev_error += 2 * M_PI;
    } else if (elev_error > M_PI){
      elev_error -= 2 * M_PI;
    }
    return std::make_tuple(target_azi_pred, target_elev_pred, azi_error, elev_error, est_pos);
}

bool RWSAutonomy::step_autonomy(double t, double dt) {
  // Select target
  double min_dist = std::numeric_limits<double>::infinity();
  int target_id = -1;
  StatePtr pTargetState;
  tie(min_dist, target_id, pTargetState) = getTarget();

  // Weapon readying
  this->wpn_countdown_ = this->wpn_countdown_ - dt;

  if (target_id >= 0){
    Eigen::Vector3d tgt_pos = pTargetState->pos();
    Eigen::Vector3d tgt_vel = pTargetState->vel();
    double target_azi_pred, target_elev_pred, azi_error, elev_error;
    Eigen::Vector3d est_pos;
    std::tie(target_azi_pred, target_elev_pred, azi_error, elev_error, est_pos) = getAim(min_dist, tgt_pos, tgt_vel, dt);

    auto aim_msg = std::make_shared<sc::Message<std::pair<double, double>>>();
    aim_msg->data = std::make_pair(target_azi_pred, target_elev_pred);
    this->pub_turret_aimpoint_->publish(aim_msg);
    // cout << "TARGET ID: " << target_id << " MIN DIST: " << min_dist << endl;
    // cout << this->wpn_countdown_ << endl;
    // Fire at target
    // Determine if weapon is ready and target in range, if it is then fire.
    if (this->wpn_countdown_ <= 0){
        if (min_dist <= this->weapon_range_){
            // if ((abs(azi_error) < azi_fire_env_) && (abs(elev_error) < elev_fire_env_)){
                this->wpn_countdown_ = this->fire_period_;
                // source id, source location, destination location, time, speed, bullet type
                auto fire_msg = std::make_shared<sc::Message<BulletData>>();
                fire_msg->data = std::make_tuple(parent_->id().id(), state_->pos(), est_pos, t, bullet_speed_, bullet_type_);
                // fire_msg->data = std::make_tuple(parent_->id().id(), state_->pos(), tgt_pos, t, bullet_speed_, bullet_type_);
                this->pub_fire_bullet_->publish(fire_msg);
                cout << "FIRE" << endl;
            // }
        }
    }
  }
  return true;
}
} // namespace autonomy
} // namespace scrimmage
