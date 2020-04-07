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

#ifndef INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_AUTONOMY_PLAYERFOLLOWBEHAVIOR_PLAYERFOLLOWBEHAVIOR_H_
#define INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_AUTONOMY_PLAYERFOLLOWBEHAVIOR_PLAYERFOLLOWBEHAVIOR_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/math/State.h>

#include <Eigen/Dense>

#include <map>
#include <string>
#include <memory>
namespace sc = scrimmage;

namespace scrimmage {
namespace autonomy {
class PlayerFollowBehavior : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
     int desired_alt_idx_ = 0;
     int desired_speed_idx_ = 0;
     int desired_heading_idx_ = 0;
     int min_dist = 0;
     scrimmage::State vehicle_broadcast_;
     int drone_id = 0;
     std::string droneId_;
     
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_AUTONOMY_PLAYERFOLLOWBEHAVIOR_PLAYERFOLLOWBEHAVIOR_H_
