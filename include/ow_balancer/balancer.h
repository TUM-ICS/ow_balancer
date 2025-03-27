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

#ifndef OPEN_WALKER_BALANCER_H
#define OPEN_WALKER_BALANCER_H

#include <ow_core/interfaces/i_balancer.h>

#include <ow_core/math.h>
#include <ow_core/algorithms.h>

#include <ow_balancer/dcm_tracker.h>
#include <ow_balancer/zmp_tracker.h>
#include <ow_balancer/imu_stabilizer.h>
#include <ow_balancer/com_tracker.h>

/*!
 * \brief Open Walker balancer module namespace. These classes implement the
 * different componets of the balance controller.
 */
namespace ow_balancer
{

/*!
 * \brief The Balancer class
 *
 * This class calculates hip offsets to minimize the effects of external forces
 * in the LIPM model. It is bassed on Capture Point tracking by means of ZMP
 * target adjustment motions.
 */
class Balancer :
  public ow::IBalancer
{
public:
  typedef ow::IBalancer Base;
  typedef ow::StateIntegrator<ow::CartesianState> Integrator;

protected:
  ow::Parameter parameter_;       //!< configuration of this module
  bool  enabled_;                 //!< Enable Balancer module.
  bool  use_imu_;                 //!< Enable IMU compensation.
  ow::Scalar  freq_;              //!< Update frequency.
  ow::Vector3 hip_offsets_;       //!< Maximum hip offset [x,y,z]
  ow::Scalar  hip_offset_decay_;  //!< Hip offsets decay \f$ in [0,1] \f$.

  // Internal variables.     
  ow::CartesianState X_off_com_;          //!< Hip offset state.
  ow::LinearState zmpD_w_;       //!< ZeroMomentPointState to keep balance

  // algorithms
  std::unique_ptr<Integrator> integrator_;
  
  // Internal modules.
  ow_balancer::DCMTracker dcm_tracker_;
  ow_balancer::ZMPTracker zmp_tracker_;
  ow_balancer::IMUStabilizer imu_stabilizer_;
  ow_balancer::COMTracker com_tracker_;

public:

  /*!
   * \brief Balancer
   *      Default constructor.
   */
  Balancer();

  /*!
   * \brief ~Balancer
   */
  virtual ~Balancer();

  /*!
   * \brief update
   * \param flags
   * \param reff_dcm
   * \param com_des
   * \return
   */
  void update(
    ow::Flags& flags,
    const ow::LinearState& ref_zmp,
    const ow::LinearState& zmp,
    const ow::LinearState& ref_dcm,
    const ow::LinearState& dcm,
    const ow::ImuSensor& imu,
    const ow::CartesianState& Xref_com_w,
    const ow::CartesianState& X_com_w,
    const ow::CartesianState& X_l_w,
    const ow::CartesianState& X_r_w);

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState offset on the Com Frame expressed in the CoM frame
  */
  virtual const ow::CartesianState& Xoff_com() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *   ZeroMomentPointState to keep balance and track the desired DCM, expressed
  *   in the world frame.
  */
  virtual const ow::LinearState& ZMPd_w() const;

  /**
   * @brief Ouput port function
   * 
   * @return const ow::Point2d& The support polygon in world frame
   */
  virtual const ow::Points2d& supportPolygon_w() const;

protected:

  /*!
   * \brief Initialization of Balancer module.
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);
};

}

#endif // OPEN_WALKER_BALANCER_H
