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

#include <new-scrimmage-plugin/plugins/sensor/VehicleLocationBroadcaster/VehicleLocationBroadcaster.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor,
                scrimmage::sensor::VehicleLocationBroadcaster,
                VehicleLocationBroadcaster_plugin)

namespace scrimmage {
namespace sensor {

VehicleLocationBroadcaster::VehicleLocationBroadcaster() {
}

void VehicleLocationBroadcaster::init(std::map<std::string, std::string> &params) {
    pub_ = advertise("GlobalNetwork", "VehicleLocationBroadcaster");
}

bool VehicleLocationBroadcaster::step() {
    // Make a copy of the current state
    sc::State ns = *(parent_->state_truth());

    // Create a message to hold the modified state
    auto msg = std::make_shared<sc::Message<sc::State>>();

    // Add noise to the three scalars in the 3D position vector.
    // for (int i = 0; i < 3; i++) {
    //     msg->data.pos()(i) = ns.pos()(i) + (*pos_noise_[i])(*gener_);
    // }

    msg->data = ns;

    // Return the sensor message.
    pub_->publish(msg);
    return true;
}
} // namespace sensor
} // namespace scrimmage
