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

#ifndef INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_AUTONOMY_RWSAUTONOMY_RWSAUTONOMY_H_
#define INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_AUTONOMY_RWSAUTONOMY_RWSAUTONOMY_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class RWSAutonomy : public scrimmage::Autonomy {
 public:
    enum class BulletType {
      regular
    };

    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    PublisherPtr pub_fire_bullet_, pub_turret_aimpoint_;
    double weapon_range_ = 100;
    double wpn_countdown_ = 0.0;
    double fire_rate_ = 0.5; // Hz
    double fire_period_ = 1/fire_rate_; // sec
    // double pk_ = 0.1;
    double azi_fire_env_ = 0.0;
    double elev_fire_env_ = 0.0;
    int target_azi_idx_ = 0;
    int target_elev_idx_ = 0;
    double bullet_speed_ = 50000.0;
    BulletType bullet_type_ = BulletType::regular;

    std::tuple<double, int, StatePtr> getTarget();
    std::tuple<double, double, double, double, Eigen::Vector3d> getAim(double dist, Eigen::Vector3d pos, Eigen::Vector3d vel, double dt);
};
} // namespace autonomy

namespace weapons {
  // source id, source location, destination location, time, speed, bullet type
  typedef std::tuple<int, Eigen::Vector3d, Eigen::Vector3d, double, double, autonomy::RWSAutonomy::BulletType> BulletData;
  const std::map<autonomy::RWSAutonomy::BulletType, std::pair<double, double>> pk_table = {
  { autonomy::RWSAutonomy::BulletType::regular, std::make_pair(0.15, 0.5) }  // pk, blast radius
};

}

} // namespace scrimmage
#endif // INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_AUTONOMY_RWSAUTONOMY_RWSAUTONOMY_H_
