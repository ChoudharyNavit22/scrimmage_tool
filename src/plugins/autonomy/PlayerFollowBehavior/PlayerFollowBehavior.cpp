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

#include <new-scrimmage-plugin/plugins/autonomy/PlayerFollowBehavior/PlayerFollowBehavior.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/math/StateWithCovariance.h>
#include <scrimmage/math/Angles.h>

// Include the Shape header and some protobuf helper functions
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h> // scrimmage::set()

#include <iostream>
#include <string>
#include <cmath>
#include <cfloat>
#include <vector>
#include <array>
#include <thread>
#include <mutex>

#include <limits>

using std::cout;
using std::endl;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::PlayerFollowBehavior,
                PlayerFollowBehavior_plugin)

namespace scrimmage {
namespace autonomy {

void PlayerFollowBehavior::init(std::map<std::string, std::string> &params) {
    
     // Entities are number sequentially from 1 by SCRIMMAGE based on the order
     // they are instantiated in the mission file.
     // Possibly there is a way to query SCRIMMAGE to find the id of the first
     // UAS entity, but here we take a shortcut and pass in the number of UAS
     // and the ID of the first UAS.  This should be fixed in future to make it
     // scalable and less brittle.
     int first_drone_id = sc::get<int>("first_drone_id", params, 1);
     int num_drones = sc::get<int>("num_drones", params, 1);

     // Here the UAS are renumbered 1 to M
     drone_id = parent_->id().id() - first_drone_id + 1;
     cout << "Drone ID: " << drone_id << endl;

     // Initialise gains for acceleration controller
     K_p = sc::get<double>("K_p", params, 1);
     K_v = sc::get<double>("K_v", params, 1);

     // Defined in paper C. Kinematic Reduction
     theta = (2.0*drone_id - num_drones - 1.0) / (2.0*num_drones - 2.0);
     cout << "Theta: " << theta << endl;
     distance_from_target = sc::get<double>("distance_from_target", params, 5.0);
     desired_altitude = sc::get<double>("desired_altitude", params, 5.0);

     acc_x_idx_ = vars_.declare(VariableIO::Type::acceleration_x, VariableIO::Direction::Out);
     acc_y_idx_ = vars_.declare(VariableIO::Type::acceleration_y, VariableIO::Direction::Out);
     acc_z_idx_ = vars_.declare(VariableIO::Type::acceleration_z, VariableIO::Direction::Out);
     heading_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
     vars_.output(acc_x_idx_, 0);
     vars_.output(acc_y_idx_, 0);
     vars_.output(acc_z_idx_, 0);

}

bool PlayerFollowBehavior::step_autonomy(double t, double dt) {
    
    // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.
    int follow_id_ = -1;
    double min_dist = std::numeric_limits<double>::infinity();

     for (auto &kv : *contacts_) {

         int contact_id = kv.first;
         sc::Contact &contact = kv.second;

         // Skip if this contact is on the same team
         if (contact.id().team_id() == parent_->id().team_id()) {
             continue;
         }

         // Calculate distance to entity
         double dist = (contact.state()->pos() - state_->pos()).norm();

         if (dist < min_dist) {
             // If this is the minimum distance, save distance and reference to
             // entity
             min_dist = dist;
             follow_id_ = contact_id;
         }
     }

     // ************ Currently hard-coded desired target heading ***************
     double desired_target_heading = Angles::deg2rad(-60.0);

     // Head toward entity on other team
     if (contacts_->count(follow_id_) > 0) {
         // Get a reference to the entity's state.
         sc::StatePtr ent_state = contacts_->at(follow_id_).state();

         // Calculate desired position in formation
         double alpha = desired_target_heading + M_PI + theta;
         Eigen::Vector3d desired_pos = ent_state->pos() + distance_from_target*Eigen::Vector3d(cos(alpha), sin(alpha), 0);

         Eigen::Vector3d delta_pos = desired_pos - state_->pos();
         Eigen::Vector3d delta_vel = ent_state->vel() - state_->vel(); // match velocity of vehicle
         Eigen::Vector3d acc = K_p * delta_pos + K_v * delta_vel;
         vars_.output(acc_x_idx_, acc(0));
         vars_.output(acc_y_idx_, acc(1));
         vars_.output(acc_z_idx_, 0.0);
         vars_.output(heading_idx_, desired_target_heading);
    }

     return true;
}
} // namespace autonomy
} // namespace scrimmage
