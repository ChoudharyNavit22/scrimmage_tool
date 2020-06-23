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

#ifndef INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_INTERACTION_DESTROYTARGETS_DESTROYTARGETS_H_
#define INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_INTERACTION_DESTROYTARGETS_DESTROYTARGETS_H_

#include <new-scrimmage-plugin/plugins/autonomy/RWSAutonomy/RWSAutonomy.h>
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Shape.h>

#include <map>
#include <list>
#include <string>


namespace scrimmage {
namespace interaction {

class DestroyTargets : public scrimmage::EntityInteraction {
 public:
    DestroyTargets();
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t, double dt) override;
 protected:

    std::list<EntityPtr> ents_;
    std::list<std::tuple<Eigen::Vector3d, double, double, double, autonomy::RWSAutonomy::BulletType>> bulletlist_;
    RandomPtr random_;
    double pk_;

    void draw_sphere(double t, double dt, Eigen::Vector3d pos, double radius);

 private:
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_NEW_SCRIMMAGE_PLUGIN_PLUGINS_INTERACTION_DESTROYTARGETS_DESTROYTARGETS_H_
