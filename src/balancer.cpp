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

#include <ow_balancer/balancer.h>

namespace ow_balancer
{

  Balancer::Balancer() : 
    Base("balancer"),
    X_off_com_(ow::CartesianState::Zero()),
    zmpD_w_(ow::LinearState::Zero())
  {
  }

  Balancer::~Balancer()
  {
  }

  bool Balancer::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // get global ow parameter
    ow::Scalar frequency = parameter.get<ow::Scalar>("loop_rate");

    // load module parameter
    parameter_.add<bool>("enabled", false);
    parameter_.add<ow::Vector3>("hip_offset_limits");
    parameter_.add<ow::Scalar>("hip_offset_decay", 1.0);

    parameter_.add<ow::Scalar>("dcm_kxi", 0.0);
    parameter_.add<ow::Scalar>("dcm_kxp", 0.0);

    parameter_.add<ow::Scalar>("zmp_kd", 0.0);
    parameter_.add<ow::Scalar>("zmp_kp", 0.0);
    parameter_.add<ow::Scalar>("zmp_dead_zone", 0.0);
    parameter_.add<ow::Vector2>("footprint_x");
    parameter_.add<ow::Vector2>("footprint_y");

    parameter_.add<bool>("use_imu", false);
    parameter_.add<ow::Vector3>("Ko_imu");

    parameter_.add<ow::Vector3>("gains_com/Kp");
    parameter_.add<ow::Vector3>("gains_com/Kd");
    
    if (!parameter_.load(nh, "balancer"))
    {
      ROS_ERROR("%s::initialize: Config loading failed.",
                Base::name().c_str());
      return false;
    }

    // init members
    parameter_.get("enabled", enabled_);
    parameter_.get("use_imu", use_imu_);
    parameter_.get("hip_offset_limits", hip_offsets_);
    parameter_.get("hip_offset_decay", hip_offset_decay_);

    dcm_tracker_.init(parameter_);
    imu_stabilizer_.init(parameter_);
    zmp_tracker_.init(parameter_);
    com_tracker_.init(parameter_, frequency);
    integrator_.reset(new Integrator(1. / frequency, false));

    return true;
  }

  void Balancer::update(
      ow::Flags &flags,
      const ow::LinearState &zmpRef_w,
      const ow::LinearState &zmp_w,
      const ow::LinearState &dcmRef_w,
      const ow::LinearState &dcm_w,
      const ow::ImuSensor &imu,
      const ow::CartesianState &Xref_com_w,
      const ow::CartesianState &X_com_w,
      const ow::CartesianState &X_l_w,
      const ow::CartesianState &X_r_w)
  {
    if(!enabled_ || !flags.hasGroundContact() || flags.state() == flags.HOMEING)
    {
      // need to be enabled, have ground contact and not in homing state
      return;
    }

    // velocity offset in com frame
    ow::CartesianVelocity xP_off_com = ow::CartesianVelocity::Zero();

    // Track the DCM
    zmpD_w_.pos() = dcm_tracker_.update(zmpRef_w, dcmRef_w, dcm_w, X_l_w.pos(), X_r_w.pos(), flags);

    // Track the ZMP.
    xP_off_com += zmp_tracker_.update(zmpD_w_, zmp_w);

    // Track COM height
    xP_off_com += com_tracker_.update(Xref_com_w, X_com_w);

    // IMU posture compensation.
    if (use_imu_)
    {
      xP_off_com += imu_stabilizer_.update(imu, Xref_com_w.pos().angular());
    }

    // transform from world frame to com frame
    X_off_com_.vel() = ow::changeRefFrame(xP_off_com, Xref_com_w.pos().inverse());
    
    // integrate the cartesian velocity to a position in com frame
    integrator_->update(X_off_com_);

    // check the limits on position part
    ow::clampSetZero(
        X_off_com_.pos().linear(), 
        X_off_com_.vel().linear(), -hip_offsets_, hip_offsets_);

    // apply decay on position
    X_off_com_.pos().linear() = hip_offset_decay_ * X_off_com_.pos().linear();
  }

  const ow::CartesianState &Balancer::Xoff_com() const
  {
    return X_off_com_;
  }

  const ow::LinearState &Balancer::ZMPd_w() const
  {
    return zmpD_w_;
  }

  const ow::Points2d& Balancer::supportPolygon_w() const
  {
    return dcm_tracker_.supportPolygon();
  }

} // namespace ow_balancer
