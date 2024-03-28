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

#ifndef ROBOTIQ_2F_GRIPPER_CLIENT_BASE_H
#define ROBOTIQ_2F_GRIPPER_CLIENT_BASE_H

#include <ros/ros.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <string>

namespace robotiq_2f_gripper_control
{

class Robotiq2FGripperClientBase
{
public:
    typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_output GripperOutput;
    typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_input GripperInput;

    virtual void init(ros::NodeHandle nh) {}

    /**
     * \brief Write the given set of control flags to the memory of the gripper
     *
     * @param[in] output The set of output-register values to write to the gripper
     */
    virtual bool writeOutputs(const std::string& output) = 0;

    /**
     * \brief Reads set of input-register values from the gripper.
     * \return The gripper input registers as read from the controller IOMap
     */
    virtual std::string readInputs() const = 0;


    virtual ~Robotiq2FGripperClientBase() {}

protected:
    Robotiq2FGripperClientBase() {}

};

} //end namespace robotiq_2f_gripper_control

#endif // ROBOTIQ_2F_GRIPPER_CLIENT_BASE_H
