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

#ifndef ROBOTIQ_2F_GRIPPER_ROS_H
#define ROBOTIQ_2F_GRIPPER_ROS_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/robotiq_2f_gripper_api.h>

namespace robotiq_2f_gripper_control
{
class Robotiq2FGripperROS
{
public:
    Robotiq2FGripperROS(ros::NodeHandle& nh,
                  boost::shared_ptr<robotiq_2f_gripper_control::Robotiq2FGripperAPI> driver, std::vector<std::string> joint_names,
                  ros::Duration desired_update_freq);

    void publish();

    void handleRawCmd(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_output::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    boost::shared_ptr<robotiq_2f_gripper_control::Robotiq2FGripperAPI> driver_;

    //! Topics
    ros::Publisher input_status_pub_;
    ros::Subscriber output_sub_;

    //! Settings
    ros::Duration desired_update_freq_;

    robotiq_2f_gripper_control::Robotiq2FGripper_robot_input input_status_msg_;

};
} //end namespace robotiq_2f_gripper_control

#endif // ROBOTIQ_2F_GRIPPER_ROS_H
