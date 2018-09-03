///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 *  gazebo_pal_hey5.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 6/3/2015
 *      Author: luca
 *
 * \brief A plugin for gazebo for controlling the pal hey5 underactuated hand in simulation
 * \author  Luca Marchionni (luca.marchionni@pal-robotics.com)
 */

#ifndef GAZEBO_PAL_HEY5_H
#define GAZEBO_PAL_HEY5_H


// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <control_toolbox/pid.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboPalHey5 : public ModelPlugin {

    public:
      GazeboPalHey5();
      ~GazeboPalHey5();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();

    private:

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string actuated_joint_name_;

      std::vector<std::string> virtual_joint_names_;

      physics::JointPtr actuated_joint_;

      std::vector<physics::JointPtr> virtual_joints_;
      typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
      std::vector<PidPtr> pids_;

      std::vector<double> scale_factors_;

      math::Angle actuator_angle_;

      std::string robot_namespace_;

  };

}

#endif // GAZEBO_PAL_HEY5_H
