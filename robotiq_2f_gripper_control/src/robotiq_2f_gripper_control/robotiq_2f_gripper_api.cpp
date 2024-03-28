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

#include "robotiq_2f_gripper_control/robotiq_2f_gripper_api.h"

using namespace robotiq_2f_gripper_control;
using namespace robotiq;

Robotiq2FGripperAPI::Robotiq2FGripperAPI(boost::shared_ptr<Robotiq2FGripperClientBase> base)
    :base_(base)
{
    pos_to_ticks_ = 200;
    pos_offset_ = 0;
    vel_to_ticks_ = 2.94;
    vel_offset_ = 22;
    force_to_ticks_ = 5.7;
    force_offset_ = 15;
    cur_to_ticks_ = 10;

    //! Get current status
    read();
    command_.rACT = status_.gACT;
    command_.rGTO = status_.gGTO;
    command_.rPR = status_.gPR;
    cmdBuf = sendData;
    recvBuf = recvData;
}

void Robotiq2FGripperAPI::setInitialization(InitializationMode mode)
{
    std::string command = "SET ACT " + std::to_string(mode) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setActionMode(ActionMode mode)
{
    std::string command = "SET GTO " + std::to_string(mode) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setEmergencyRelease(EmergencyRelease release)
{
    std::string command = "SET STR " + std::to_string(release) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setPosition(const double &pos)
{
    std::string command = "SET POS " + std::to_string(pos_to_ticks_*(pos - pos_offset_)) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setVelocity(const double &vel)
{
    std::string command = "SET SPE " + std::to_string(vel_to_ticks_*(vel - vel_offset_)) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setForce(const double &f)
{
    std::string command = "SET FOR " + std::to_string(force_to_ticks_*(f - force_offset_)) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setSid(const double &sid)
{
    std::string command =  "SET SID " + std::to_string(sid) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::setRaw(const Robotiq2FGripperClientBase::GripperOutput &raw)
{
    std::string command = "SET POS " + std::to_string(raw.rPR) + " SPE " + std::to_string(raw.rSP) + " FOR " + std::to_string(raw.rFR) + "\n";
    write(command);
}

void Robotiq2FGripperAPI::getPosition(double *pos) const
{
    std::string command = "GET POS\n";
    int gPO = 0;
    *pos = (double)gPO/pos_to_ticks_ + pos_offset_;
}

void Robotiq2FGripperAPI::getPositionCmd(double *pos) const
{
    std::string command = "GET PRE\n";
    //*pos = (double)status_.gPR/pos_to_ticks_ + pos_offset_;
}

void Robotiq2FGripperAPI::getCurrent(double *cur) const
{
    ;
}

void Robotiq2FGripperAPI::getSid(double *sid)
{
    std::string command = "GET SID\n";
    getData(command);
}

void Robotiq2FGripperAPI::getGripperStatus(InitializationMode *gACT, ActionMode *gGTO, MotionStatus *gSTA) const
{
    std::string command_1 = "GET STA\n";
    std::string command_2 = "GET ACT\n";
    std::string command_3 = "GET GTO\n";
}

void Robotiq2FGripperAPI::getFaultStatus(FaultStatus *gFLT) const
{
    std::string command = "GET FLT\n";
}

void Robotiq2FGripperAPI::getRaw(Robotiq2FGripperClientBase::GripperInput *raw) const
{
    *raw = status_;
}

void Robotiq2FGripperAPI::getCommandPos(double *pos) const
{
    *pos = (double)command_.rPR/pos_to_ticks_ + pos_offset_;
}

bool Robotiq2FGripperAPI::isInitialized()
{
    return ((InitializationMode)status_.gACT == INIT_ACTIVATION);
}

bool Robotiq2FGripperAPI::isHalted()
{
    return ((ActionMode)status_.gGTO == ACTION_STOP);
}

bool Robotiq2FGripperAPI::isMoving()
{
    return (status_.gSTA == MOTION_STARTED);
}

bool Robotiq2FGripperAPI::isEmergReleaseComplete()
{
    return ((FaultStatus)status_.gFLT == ERROR_AUTOMATIC_RELEASE_COMPLETE);
}

void Robotiq2FGripperAPI::getData(std::string request)
{
    write(request);
    read();
}

void Robotiq2FGripperAPI::decode(std::string &input)
{
    size_t spacePos = input.find(' ');

    std::string valueStr = input.substr(spacePos + 1);
    std::string variableName = input.substr(0, spacePos);

    // Convert the value string to an integer
    int value;
    std::istringstream iss(valueStr);
    iss >> value;

}

void Robotiq2FGripperAPI::read()
{
    std::string response = base_->readInputs();
    decode(response);
}

void Robotiq2FGripperAPI::write(std::string &command)
{
    base_->writeOutputs(command);
}
