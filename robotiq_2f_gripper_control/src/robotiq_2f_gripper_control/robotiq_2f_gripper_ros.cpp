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

#include "robotiq_2f_gripper_control/robotiq_2f_gripper_ros.h"

using namespace robotiq_2f_gripper_control;

Robotiq2FGripperROS::Robotiq2FGripperROS(ros::NodeHandle &nh, boost::shared_ptr<Robotiq2FGripperAPI> driver, std::vector<std::string> joint_names, ros::Duration desired_update_freq)
    :nh_(nh)
    ,driver_(driver)
    ,desired_update_freq_(desired_update_freq)
{
    //! advertise topics
    input_status_pub_ = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>("input", 10);

    //! subscribers
    output_sub_ = nh.subscribe("output", 10, &Robotiq2FGripperROS::handleRawCmd, this);

    if(joint_names.size() != 1)
    {
        ROS_FATAL("Joint name size must be 1");
    }
}

void Robotiq2FGripperROS::publish()
{
    driver_->getRaw(&input_status_msg_);
    input_status_pub_.publish(input_status_msg_);
}

void Robotiq2FGripperROS::handleRawCmd(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_output::ConstPtr &msg)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_raw_cmd");
    driver_->setRaw(*msg);
}
