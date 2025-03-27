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

#ifndef OPEN_WALKER_DCM_TRACKER_H
#define OPEN_WALKER_DCM_TRACKER_H

#include <ow_core/types.h>
#include <ow_core/common/parameter.h>
#include <ow_core/math.h>
#include <ow_core/geometry/geometry_2d.h>

namespace ow_balancer
{

/*!
 * \brief The DCMTracker class
 *
 * Here
 */
class DCMTracker
{

protected:

  // Parameters.
  ow::Scalar  w_;                   //!< LIPM model time constant.

  // Gains.
  ow::Scalar kxi_;                  //!< Proportional Gain for Capture point tracker.
  ow::Scalar kxp_;                  //!< Integral gain for capture point tracker.
  ow::Scalar kxz_;                  //!< zmp error

  ow::Vector3 i_err;               //!< Integral term on zmp

  // Variables.
  ow::LinearPosition zmp_des_;     //!< Desired ZMP for tracking the reference DCM.

  ow::LinearPosition lower_limit_; 
  ow::LinearPosition upper_limit_;

  ow::Points2d footprint_f_;        //!< footprint wrt to foot frame
  ow::Points2d supportpolygon_w_;   //!< support polygon wrt to world frame

public:

  /*!
   * \brief DCMTracker
   *      Default constructor.
   */
  DCMTracker();

  ~DCMTracker();

  /*!
   * \brief init
   * \param kxi_gain
   * \return
   */
  bool init(ow::Parameter& parameter);

  /*!
   * \brief trackCP
   * \param ref_zmp
   * \param ref_dcm
   * \param dcm
   * \return
   */
  const ow::LinearPosition& update(const ow::LinearState &ref_zmp,
                                      const ow::LinearState& ref_dcm,
                                      const ow::LinearState& dcm,
                                      const ow::CartesianPosition& X_l_w,
                                      const ow::CartesianPosition& X_r_w,
                                      const ow::Flags& flags);

  /*!
   * \brief zmpDes
   * \return
   */
  const ow::LinearPosition& zmpDes() const;
  
  /**
   * @brief supportpolygon
   * 
   * @return const ow::Point2d& 
   */
  const ow::Points2d& supportPolygon() const;

private: 
 
  void clampToSupportFoot(
    ow::LinearPosition& zmp, const ow::CartesianPosition& X_f_w);

  void clampToBothFeet(
    ow::LinearPosition& zmp, const ow::CartesianPosition& X_l_w,
    const ow::CartesianPosition& X_r_w);

};

}

#endif // OPEN_WALKER_DCM_TRACKER_H
