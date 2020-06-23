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

#include <new-scrimmage-plugin/plugins/interaction/DestroyTargets/DestroyTargets.h>

#include <new-scrimmage-plugin/plugins/autonomy/RWSAutonomy/RWSAutonomy.h>


#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h> // scrimmage::set()

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;
using namespace scrimmage::weapons;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::DestroyTargets,
                DestroyTargets_plugin)

namespace scrimmage {
namespace interaction {

DestroyTargets::DestroyTargets() {
}

bool DestroyTargets::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    // random_ = ents_.front()->random();

    auto destroy_target_cb = [&] (scrimmage::MessagePtr<std::pair<int, float>> msg) {  //id, pkill
        if (ents_.empty()) {
            return true;
        }
        for (auto const& i : ents_) {
          if (i->id().id() == (msg->data).first){
            if (i->is_alive()){
              random_ = i->random();
              if(random_->rng_uniform(0.0, 1.0) <= (msg->data).second){
                 i->collision();
              }
            }
          }
        }
        return true;
    };
    subscribe<std::pair<int, float>>("GlobalNetwork", "DestroyTarget", destroy_target_cb);

    auto fire_bullet_cb = [&] (scrimmage::MessagePtr<BulletData> msg) {
      // cout << "FIRE MSG RECEIVED" << endl;
      if (ents_.empty()) {
          return true;
      }
      // source id, source location, destination location, time, speed, bullet type

      // Determine time to impact
      Eigen::Vector3d source_pos = std::get<1>(msg->data);
      Eigen::Vector3d tgt_pos = std::get<2>(msg->data);
      double time_when_fired = std::get<3>(msg->data);
      double speed = std::get<4>(msg->data);
      autonomy::RWSAutonomy::BulletType bullet_type = std::get<5>(msg->data);

      double distance = (tgt_pos - source_pos).norm();
      double tti = distance / speed;
      cout << "Time when fired: " << time_when_fired << endl;
      cout << "Time message received: " << time_->t() << endl;

      double pk = 0.0;
      double blast_radius = 0.0;
      std::tie(pk, blast_radius) = pk_table.at(bullet_type);

      // Add location, time to impact, pk and bullet type to list
      std::tuple<Eigen::Vector3d, double, double, double, autonomy::RWSAutonomy::BulletType> bullet_info = std::make_tuple(std::get<2>(msg->data), tti + time_when_fired, pk, blast_radius, bullet_type);
      bulletlist_.push_back(bullet_info);
      //TODO determine if there is a time step issue and whether should be able to destroy targets here


      // for (auto const& i : ents_) {
        // if (i->id().id() == (msg->data).first){
        //   if (i->is_alive()){
        //     random_ = i->random();
        //     if(random_->rng_uniform(0.0, 1.0) <= (msg->data).second){
        //        i->collision();
        //     }
        //   }
        // }
        // cout << "TIME: " << time_->t() << endl;
      // }
      return true;
    };
    subscribe<BulletData>("GlobalNetwork", "FireBullet", fire_bullet_cb);
    return true;
}

void DestroyTargets::draw_sphere(double t, double dt, Eigen::Vector3d pos, double radius) {
   // if (sphere_shape_ == nullptr) {
  scrimmage_proto::ShapePtr sphere_shape = sc::shape::make_sphere(
           pos, radius,
           Eigen::Vector3d(255, 185, 120), 0.6);
   // }
   // sc::set(sphere_shape->mutable_sphere()->mutable_center(), parent_->state_truth()->pos());
   draw_shape(sphere_shape);
}

bool DestroyTargets::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    // cout << "DestroyTargets : " << t << endl;
    if (ents.empty()) {
        return true;
    }
    ents_ = ents;

    std::list<std::tuple<Eigen::Vector3d, double, double, double, autonomy::RWSAutonomy::BulletType>>::iterator it = bulletlist_.begin();
    while (it != bulletlist_.end())
    {
      double impact_time = std::get<1>(*it);
      if(impact_time < t){
        cout << "Detonation" << endl;
        Eigen::Vector3d detonation_location = std::get<0>(*it);
        double pk = std::get<2>(*it);
        double blast_radius = std::get<3>(*it);
        draw_sphere(t, dt, detonation_location, blast_radius);

        it = bulletlist_.erase(it);
        // Find all ents within blast radius
        double dist = 0;
        for (auto const& entity : ents_) {
          Eigen::Vector3d pos = entity->state()->pos();
          dist = (detonation_location - pos).norm();
          if (dist <= blast_radius){
            RandomPtr random = entity->random();
            if(random->rng_uniform(0.0, 1.0) <= pk){
              cout << "HIT!" << endl;
              entity->collision();
            }
          }
            // if (entity->is_alive()){
            //   random_ = entity->random();
            //   if(random_->rng_uniform(0.0, 1.0) <= (msg->data).second){
            //      i->collision();
            //   }
            // }
          // }
        }
      }else{
        it++;
      }
    }

    return true;
}
} // namespace interaction
} // namespace scrimmage
