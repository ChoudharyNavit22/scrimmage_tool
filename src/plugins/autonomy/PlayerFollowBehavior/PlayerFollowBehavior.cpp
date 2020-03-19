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
     double initial_speed = sc::get<double>("initial_speed", params, 15);
     desired_alt_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
     desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
     desired_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);

     vars_.output(desired_speed_idx_, initial_speed);
     vars_.output(desired_alt_idx_, state_->pos()(2));
     vars_.output(desired_heading_idx_, state_->quat().yaw());

    auto state_cb = [&](auto &msg) {
        vehicle_broadcast_ = msg->data;
        // for (int i = 0; i < 3; i++) {
        //     cout << "position: "<< i << " ===> "<< vehicle_broadcast_.pos()(i) << endl;
        // }
    };

    subscribe<sc::State>("GlobalNetwork", "VehicleLocationBroadcaster", state_cb);

}

bool PlayerFollowBehavior::step_autonomy(double t, double dt) {
    // Find nearest entity on other team. Loop through each contact, calculate
     // distance to entity, save the ID of the entity that is closest.
     int follow_id_ = -1;
     double min_dist = std::numeric_limits<double>::infinity();
     //bool first_interact = 0;
     for (auto &kv : *contacts_) {

         int contact_id = kv.first;
         sc::Contact &contact = kv.second;

         // Skip if this contact is on the same team
         if (contact.id().team_id() == parent_->id().team_id()) {
             continue;
         }

        //  if(!first_interact){
        //     // Create the GenerateEntity message
        //     auto msg = std::make_shared<Message<scrimmage_msgs::GenerateEntity>>();
        //     sc::set(msg->data.mutable_state(), contact.id().team_id()); // Copy the new state
        //     publish(msg); // Publish the GenerateEntity message
        //     first_interact = 1;
        //  }

         // Calculate distance to entity
         double dist = (contact.state()->pos() - state_->pos()).norm();

         if (dist < min_dist) {
             // If this is the minimum distance, save distance and reference to
             // entity
             min_dist = dist;
             follow_id_ = contact_id;
         }
     }

     // Head toward entity on other team
     if (contacts_->count(follow_id_) > 0) {
         // Get a reference to the entity's state.
         sc::StatePtr ent_state = contacts_->at(follow_id_).state();

         // Calculate the required heading to follow the other entity
         double heading = atan2(ent_state->pos()(1) - state_->pos()(1),
                                ent_state->pos()(0) - state_->pos()(0));

         // Set the heading
         vars_.output(desired_heading_idx_, heading);

         // Match entity's altitude
         vars_.output(desired_alt_idx_, 10);
     }

     return true;
}
} // namespace autonomy
} // namespace scrimmage
