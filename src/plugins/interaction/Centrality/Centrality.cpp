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

#include <new-scrimmage-plugin/plugins/interaction/Centrality/Centrality.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/parse/MissionParse.h>

#include <scrimmage/common/CSV.h>


#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::Centrality,
                Centrality_plugin)

namespace scrimmage {
namespace interaction {

Centrality::Centrality() {
}

Centrality::~Centrality(){
  std::cout << "CLOSED INTERACTION PLUGIN" << std::endl;
  csv_.close_output();
}

bool Centrality::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    UAS_comms_range_ = sc::get<double>("comms_range", plugin_params, 100.0);
    pub_degree_centrality_ = advertise("GlobalNetwork", "AvgDegCentrality");

    // Write the CSV file to the root log directory
    std::string filename = parent_->mp()->root_log_dir() + "/avg_degree_centrality.csv";

    // Create the file
    if (!csv_.open_output(filename)) {
       std::cout << "Couldn't create output file" << endl;
    }else{
       std::cout << "CREATED: " << filename << endl;
    }

    csv_.set_column_headers("t, avg_degree_centrality");

    return true;
}


bool Centrality::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (ents.empty()) {
      return true;
    }

    int num_uas = 0;
    float degree_centrality = 0.0;

    // For each UAS
    for (sc::EntityPtr uas1 : ents) {
      if (uas1->id().team_id() == 2){
        num_uas++;
        // For each UAS
        for (sc::EntityPtr uas2 : ents) {
          if (uas2->id().team_id() == 2){
            if (uas1->id().id() != uas2->id().id()){
              double dist = (uas2->state()->pos() - uas1->state()->pos()).norm();
              if (dist <= UAS_comms_range_){
                //     std::pair <int, int> msg = std::make_pair(ent->id().id(), tgt->id().id());
                //     auto shared_msg = std::make_shared<sc::Message<std::pair <int, int>>>();
                //     shared_msg->data = msg;
                //     pub_in_range_->publish(shared_msg);
                //     ent->collision();
                degree_centrality++;
              }
            }
          }
        }
      }
    }

    double avg_degree_centrality = degree_centrality/num_uas;
    auto shared_msg = std::make_shared<sc::Message<double>>();
    shared_msg->data = avg_degree_centrality;
    pub_degree_centrality_->publish(shared_msg);
    // std::cout << "Avg deg centrality: " << avg_d egree_centrality << std::endl;
    csv_.append(sc::CSV::Pairs{
     {"t", t},
     {"avg_degree_centrality", avg_degree_centrality}});
    return true;
}
} // namespace interaction
} // namespace scrimmage
