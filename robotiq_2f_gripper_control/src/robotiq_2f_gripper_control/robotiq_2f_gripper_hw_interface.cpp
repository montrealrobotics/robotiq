// Copyright (c) 2016, Toyota Research Institute. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robotiq_2f_gripper_control/robotiq_2f_gripper_hw_interface.h"

using namespace robotiq_2f_gripper_control;

Robotiq2FGripperHWInterface::Robotiq2FGripperHWInterface(ros::NodeHandle nh, boost::shared_ptr<Robotiq2FGripperAPI> driver)
    :hw_driver_(driver)
{
    double rate;
    nh.param<double>("update_rate", rate, 20);

    std::string hw_name;
    hw_name = nh.param<std::string>("hw_name", "robotiq_s");

    std::string prefix;
    std::string actuated_joint;
    prefix = nh.param<std::string>("prefix", "");

    nh.param<std::string>("actuated_joint", actuated_joint, "hande_left_finger_joint");
    joint_names_.push_back(prefix+actuated_joint);

    joint_names_ = nh.param< std::vector<std::string> >("joint_names", joint_names_);

    if(joint_names_.size()!=1)
    {
        throw std::runtime_error("There must be 1 joint name");
    }

    j_curr_pos_.resize(1, 0);
    j_curr_vel_.resize(1, 0);
    j_curr_eff_.resize(1, 0);

    j_cmd_pos_.resize(1, 0);

    hw_diagnostics_.reset(new Robotiq2FGripperDiagnostics(hw_driver_, hw_name));
    hw_ros_.reset(new Robotiq2FGripperROS(nh, hw_driver_, joint_names_, ros::Duration(1/rate)));
}

void Robotiq2FGripperHWInterface::configure(hardware_interface::JointStateInterface &joint_state_interface, hardware_interface::PositionJointInterface &joint_position_interface)
{
    //! Connect and register jonit state interface

    // Create joint state interface
    joint_state_interface.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[0],
            &j_curr_pos_[0],
            &j_curr_vel_[0],
            &j_curr_eff_[0]));


    //! Connect and register joint position interface
    // Create joint position interface
    joint_position_interface.registerHandle(hardware_interface::JointHandle(
            joint_state_interface.getHandle(joint_names_[0]),
            &j_cmd_pos_[0]));
}

void Robotiq2FGripperHWInterface::read(ros::Duration d)
{
    hw_driver_->getPosition(&j_curr_pos_[0]);
    hw_diagnostics_->update();
    hw_ros_->publish();
}

void Robotiq2FGripperHWInterface::write(ros::Duration d)
{
//    ROS_INFO("1 %f", j_cmd_pos_[0]);
//    hw_driver_->getCommandPos(&j_cmd_pos_[0]);
    hw_driver_->setPosition(j_cmd_pos_[0]);
}
