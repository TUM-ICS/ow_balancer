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

#ifndef OPEN_WALKER_IMU_STABILIZER_H
#define OPEN_WALKER_IMU_STABILIZER_H

#include <ow_core/math.h>
#include <ow_core/algorithms.h>
#include <ow_core/common/parameter.h>

namespace ow_balancer
{

/*!
 * \brief The Balancer class
 *
 * This class calculates hip offsets to minimize the effects of external forces
 * in the LIPM model. It is bassed on Capture Point tracking by means of ZMP
 * target adjustment motions.
 */
class IMUStabilizer
{

protected:

  // Gains.
  ow::Matrix3 Ko_;    //!< Rotational gain for imu compensation.

  // Internal variables.
  ow::CartesianVelocity dx_;

public:

  /*!
   * \brief Balancer
   *      Default constructor.
   */
  IMUStabilizer();

  ~IMUStabilizer();

  /*!
   * \brief Initialization of Balancer module.
   */
  bool init(const ow::Parameter& parameter);

  const ow::CartesianVelocity& update(
      const ow::ImuSensor& imu,
      const ow::AngularPosition& Qd);

  const ow::CartesianVelocity& dx() const;

};

}

#endif // OPEN_WALKER_IMU_STABILIZER_H
