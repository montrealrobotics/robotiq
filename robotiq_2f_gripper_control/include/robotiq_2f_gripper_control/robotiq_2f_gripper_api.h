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

#ifndef ROBOTIQ_2F_GRIPPER_API_H
#define ROBOTIQ_2F_GRIPPER_API_H

#include <robotiq_2f_gripper_control/robotiq_2f_gripper_client_base.h>
#include <stdint.h>
#include <string>

namespace robotiq
{
enum InitializationMode { INIT_RESET, INIT_ACTIVATION };
enum ActionMode { ACTION_STOP, ACTION_GO };
enum MotionStatus { MOTION_STARTED, MOTION_PARTIAL_STOP, MOTION_ALL_STOP, MOTION_COMPLETE };
enum FaultStatus { FAULT_NONE, FAULT_UNKNOWN_1, FAULT_UNKNOWN_2, FAULT_UNKNOWN_3,
                   NOTICE, NOTICE_ACTIVATION_DELAYED, NOTICE_MODE_DELAYED, NOTICE_ACTIVATION_NEEDED,
                   WARNING, WARNING_COMM_NOT_READY, WARNING_MODE, WARNING_AUTOMATIC_RELEASE,
                   ERROR, ERROR_ACTIVATION_FAULT, ERROR_MODE_FAULT, ERROR_AUTOMATIC_RELEASE_COMPLETE };

enum EmergencyRelease { EMERGENCY_RELEASE_IDLE, EMERGENCY_RELEASE_ENGAGED };
enum IndividualControl { IND_CONTROL_OFF, IND_CONTROL_ON };
} // end namespace robotiq

namespace robotiq_2f_gripper_control
{
using namespace robotiq;

class Robotiq2FGripperAPI
{
public:
    Robotiq2FGripperAPI(boost::shared_ptr<Robotiq2FGripperClientBase> base);

    void setInitialization(InitializationMode mode);
    void setActionMode(ActionMode mode);
    void setEmergencyRelease(EmergencyRelease release);
    void setPosition(const double &pos);
    void setVelocity(const double &vel);
    void setForce(const double &f);
    void setSid(const double &sid);
    void setRaw(const Robotiq2FGripperClientBase::GripperOutput &raw);

    void getPosition(double *pos) const;
    void getPositionCmd(double *pos) const;
    void getCurrent(double *cur) const;
    void getSid(double *sid);
    void getGripperStatus(InitializationMode *gACT,  ActionMode *gGTO, MotionStatus *gSTA) const;
    void getFaultStatus(FaultStatus *gFLT) const;
    void getRaw(Robotiq2FGripperClientBase::GripperInput *raw) const;

    void getCommandPos(double *pos) const;

    bool isInitialized();
    bool isHalted();
    bool isMoving();
    bool isEmergReleaseComplete();

    void read();
    void write(std::string &command);
    void getData(std::string request);
    void decode(std::string &input);

private:
    boost::shared_ptr<Robotiq2FGripperClientBase> base_;

    Robotiq2FGripperClientBase::GripperInput status_;
    Robotiq2FGripperClientBase::GripperOutput command_;

    //! conversions
    double pos_to_ticks_;
    double pos_offset_;
    double sci_to_ticks_;
    double sci_offset_;
    double vel_to_ticks_;
    double vel_offset_;
    double force_to_ticks_;
    double force_offset_;
    double cur_to_ticks_;

    uint8_t *cmdBuf;
    uint8_t *recvBuf;

    uint8_t recvData[100];
    uint8_t sendData[100];
    std::map<std::string, int> registerMap;

};

template <typename T>
inline T limit (double value)
{
    value = value < std::numeric_limits<T>::min() ? std::numeric_limits<T>::min() : value;
    value = value > std::numeric_limits<T>::max() ? std::numeric_limits<T>::max() : value;
    return static_cast<T>(value);
}

} //end namespace robotiq_2f_gripper_control

#endif // ROBOTIQ_2F_GRIPPER_API_H
