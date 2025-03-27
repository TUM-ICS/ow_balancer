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

#include <ow_balancer/com_tracker.h>

namespace ow_balancer
{

COMTracker::COMTracker() : 
  dx_(ow::CartesianVelocity::Zero())
{
}

COMTracker::~COMTracker()
{
}

bool COMTracker::init(const ow::Parameter& parameter, ow::Scalar frequency)
{
  // get gains
  Kp_ = parameter.get<ow::Vector3>("gains_com/Kp").asDiagonal();
  Kd_ = parameter.get<ow::Vector3>("gains_com/Kd").asDiagonal();
  return true;
}

const ow::CartesianVelocity& COMTracker::update(
    const ow::CartesianState& Xd_com_w,
    const ow::CartesianState& X_com_w)
{
  // simple pd feedback
  dx_.linear() = Kp_*(Xd_com_w.pos().linear() - X_com_w.pos().linear());
  return dx_;
}

const ow::CartesianVelocity& COMTracker::dx() const
{
  return dx_;
}

}
