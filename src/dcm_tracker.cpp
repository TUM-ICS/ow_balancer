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

#include <ow_balancer/dcm_tracker.h>

namespace ow_balancer
{

DCMTracker::DCMTracker():
  w_(0.0),
  kxp_(0.0),
  kxi_(0.0),
  i_err(ow::Vector3::Zero()),
  zmp_des_(ow::LinearPosition::Zero())
{
}

DCMTracker::~DCMTracker()
{
}

bool DCMTracker::init(ow::Parameter& parameter)
{
  ow::Vector2 foot_print_x, foot_print_y;

  // get parameter
  parameter.get("dcm_kxi", kxi_);
  parameter.get("dcm_kxp", kxp_);
  parameter.get<ow::Vector2>("footprint_x", foot_print_x);
  parameter.get<ow::Vector2>("footprint_y", foot_print_y);

  // store limits
  lower_limit_ << foot_print_x[0], foot_print_y[0], -10.0;
  upper_limit_ << foot_print_x[1], foot_print_y[1], 10.0;

  // store footprint vertices
  footprint_f_.resize(2, 4);
  footprint_f_ << 
    foot_print_x[0], foot_print_x[1], foot_print_x[1], foot_print_x[0],
    foot_print_y[0], foot_print_y[0], foot_print_y[1], foot_print_y[1];

  return true;
}

const ow::LinearPosition& DCMTracker::update(
    const ow::LinearState& ref_zmp,
    const ow::LinearState& ref_dcm,
    const ow::LinearState& dcm,
    const ow::CartesianPosition& X_l_w,
    const ow::CartesianPosition& X_r_w,
    const ow::Flags& flags)
{
  w_ = sqrt(OW_GRAVITY/dcm.pos().z());

  // dcm error
  ow::Vector3 err = ref_dcm.pos() - dcm.pos();

  // integral term
  ow::Scalar dt = 1/200.0;
  i_err += err * dt;

  // compute desired zmp
  zmp_des_ = ref_zmp.pos() - (1.0 + kxp_/w_)*err - kxi_/w_*i_err;
  zmp_des_.z() = 0.0;

  // keep the zmp in support polygon
  if(flags.walkingPhase() == ow::Flags::SINGLE_SUPPORT)                         
  {
    if(flags.supportFoot().isLeft())
    {
      // single support left
      clampToSupportFoot(zmp_des_, X_l_w);
    }
    else
    {
      // single support right
      clampToSupportFoot(zmp_des_, X_r_w);
    }
  }
  else
  {
    // double support
    clampToBothFeet(zmp_des_, X_l_w, X_r_w);                       
  }

  return zmp_des_;
}

void DCMTracker::clampToSupportFoot(ow::LinearPosition& zmp_w, const ow::CartesianPosition& X_f_w)
{
  // compute support polygon
  supportpolygon_w_ = ow::changeRefFrame(footprint_f_, X_f_w);

  // transform into foot frame
  zmp_w = X_f_w.inverse()*zmp_w;

  // clamp zmp within foot boundaries
  ow::clamp(zmp_w, lower_limit_, upper_limit_);                             

  // transform back to world
  zmp_w = X_f_w*zmp_w;
}

void DCMTracker::clampToBothFeet(
  ow::LinearPosition& zmp_w, const ow::CartesianPosition& X_l_w, const ow::CartesianPosition& X_r_w)
{
  // transform footprint to world frame
  ow::Points2d footprint_l_w = ow::changeRefFrame(footprint_f_, X_l_w);
  ow::Points2d footprint_r_w = ow::changeRefFrame(footprint_f_, X_r_w);

  // compute union polygon
  supportpolygon_w_ = ow::union_polygon_no_overlap(footprint_l_w, footprint_r_w);

  // check if in support_polygon
  if(!ow::point_polygon_intersect(supportpolygon_w_, zmp_w.head(2)))
  {
    // zmp outside, project back into polygon  
    zmp_w.head(2) = ow::closest_point_on_polygon(zmp_w.head(2), supportpolygon_w_);
  }
}

const ow::LinearPosition& DCMTracker::zmpDes() const
{
  return zmp_des_;
}

const ow::Points2d& DCMTracker::supportPolygon() const
{
  return supportpolygon_w_;
}

}
