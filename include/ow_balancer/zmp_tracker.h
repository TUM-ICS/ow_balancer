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

#ifndef OPEN_WALKER_ZMP_TRACKER_H
#define OPEN_WALKER_ZMP_TRACKER_H

#include <ow_core/types.h>
#include <ow_core/common/parameter.h>

namespace ow_balancer
{

/*!
 * \brief The ZMPTracker class
 *
 * Here
 */
class ZMPTracker
{

protected:

  // Gains.
  ow::Scalar  kp_;              //!< ZMP tracker proportional gain.
  ow::Scalar  kd_;              //!< ZMP tracker derivative gain.
  ow::Scalar  zmp_dead_zone_;   //!< ZMP dead zone.

  ow::CartesianVelocity dx_;

public:

  /*!
   * \brief ZMPTracker
   *      Default constructor.
   */
  ZMPTracker();

  /*!
   * \brief ZMPTracker
   */
  ~ZMPTracker();

  /*!
   * \brief init
   * \param kp
   * \param kd
   * \param zmp_dead_zone
   * \return
   */
  bool init(const ow::Parameter& parameter);

  /*!
   * \brief trackZMP
   * \param reff_zmp
   * \param zmp
   * \return
   */
  const ow::CartesianVelocity& update(
      const ow::LinearState& des_zmp,
      const ow::LinearState& zmp );

  const ow::CartesianVelocity& dx() const;

};

}

#endif // OPEN_WALKER_ZMP_TRACKER_H
