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

#include <ow_balancer/imu_stabilizer.h>

namespace ow_balancer
{

IMUStabilizer::IMUStabilizer():
  Ko_(ow::Matrix3::Zero()),
  dx_(ow::CartesianVelocity::Zero())
{
}

IMUStabilizer::~IMUStabilizer()
{
}

bool IMUStabilizer::init(const ow::Parameter& parameter)
{
  Ko_ = parameter.get<ow::Vector3>("Ko_imu").asDiagonal();
  return true;
}

const ow::CartesianVelocity& IMUStabilizer::update(
    const ow::ImuSensor& imu,
    const ow::AngularPosition& Qd)
{
  // compute the rotation error, expressed in the world frame
  // note: we are not considering yaw error

  // compute the yaw quaternions for desired and real orientaion
  ow::AngularPosition Qyaw = imu.Q().yawQuaternion();
  ow::AngularPosition Qd_yaw = Qd.yawQuaternion();

  // replace the real orientation with the desired one
  ow::AngularPosition Q_corr = Qd_yaw*Qyaw.inverse()*imu.Q();

  // compute the orientation error in world frame
  dx_.angular() = Ko_*ow::quaternionLogError(Qd, Q_corr);
  dx_.angular().z() = 0.0;
  return dx_;
}

const ow::CartesianVelocity& IMUStabilizer::dx() const
{
  return dx_;
}

}
