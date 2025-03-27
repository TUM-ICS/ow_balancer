/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_balancer/zmp_tracker.h>

namespace ow_balancer
{

ZMPTracker::ZMPTracker():
  kp_(0.0),
  kd_(0.0),
  zmp_dead_zone_(0.0),
  dx_(ow::CartesianVelocity::Zero())
{
}

ZMPTracker::~ZMPTracker()
{
}

bool ZMPTracker::init(const ow::Parameter& parameter)
{
  parameter.get("zmp_kd", kd_);
  parameter.get("zmp_kp", kp_);
  parameter.get("zmp_dead_zone", zmp_dead_zone_);

  return true;
}

const ow::CartesianVelocity& ZMPTracker::update(
    const ow::LinearState &des_zmp,
    const ow::LinearState& zmp )
{
  ow::LinearPosition zmp_error;
  zmp_error = zmp.pos() - des_zmp.pos();

  if(zmp_error.norm() > zmp_dead_zone_ )
  {
    // this is expressed in world
    dx_.linear() = kp_*(zmp_error) + kd_*zmp.vel();
    dx_.linear().z() = 0.0;
  }
  return dx_;
}

const ow::CartesianVelocity& ZMPTracker::dx() const
{
  return dx_;
}


}
