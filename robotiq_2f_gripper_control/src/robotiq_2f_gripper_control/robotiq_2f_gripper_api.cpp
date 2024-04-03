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
    pos_to_ticks_ = 5100;
    pos_offset_ = 0;
    vel_to_ticks_ = 2.94;
    vel_offset_ = 22;
    force_to_ticks_ = 5.7;
    force_offset_ = 15;
    cur_to_ticks_ = 10;

    sid = 0;

    // Read sid from gripper on startup

    // Populate the statusMap with default values (or values from your ROS message if available)
    statusMap["ACT"] = &Robotiq2FGripperClientBase::GripperInput::gACT;
    statusMap["GTO"] = &Robotiq2FGripperClientBase::GripperInput::gGTO;
    statusMap["POS"] = &Robotiq2FGripperClientBase::GripperInput::gPO;
    statusMap["STA"] = &Robotiq2FGripperClientBase::GripperInput::gSTA;
    statusMap["PRE"] = &Robotiq2FGripperClientBase::GripperInput::gPR;
    statusMap["OBJ"] = &Robotiq2FGripperClientBase::GripperInput::gOBJ;
    statusMap["FLT"] = &Robotiq2FGripperClientBase::GripperInput::gFLT;

}

void Robotiq2FGripperAPI::setInitialization(InitializationMode mode)
{
    std::string command2 = "SET ACT " + std::to_string(mode) + "\n";
    write(command2);
    unsigned long bytes_read_2 = read(false);
}

void Robotiq2FGripperAPI::setMotionState(MotionStatus mode)
{
    std::string command = "SET STA " + std::to_string(mode) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
}

void Robotiq2FGripperAPI::setActionMode(ActionMode mode)
{
    std::string command = "SET GTO " + std::to_string(mode) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
}

void Robotiq2FGripperAPI::setEmergencyRelease(EmergencyRelease release)
{
    std::string command = "SET STR " + std::to_string(release) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
}

void Robotiq2FGripperAPI::setPosition(const double &pos)
{
    std::string command = "SET POS " + std::to_string(int(pos_to_ticks_*(pos - pos_offset_))) + " SPE " + std::to_string(int(command_.rSP)) + " FOR " + std::to_string(int(command_.rFR)) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
    command_.rPR = (pos - pos_offset_) * pos_to_ticks_;
}

void Robotiq2FGripperAPI::setVelocity(const int &vel)
{
    std::string command = "SET SPE " + std::to_string(int(vel_to_ticks_*(vel - vel_offset_))) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
}

void Robotiq2FGripperAPI::setForce(const int &f)
{
    std::string command = "SET FOR " + std::to_string(int(force_to_ticks_*(f - force_offset_))) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
}

void Robotiq2FGripperAPI::setSid(const int &sid_p)
{
    std::string command =  "SET SID " + std::to_string(sid_p) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
}

void Robotiq2FGripperAPI::setRaw(const Robotiq2FGripperClientBase::GripperOutput &raw)
{
    std::string command = "SET POS " + std::to_string(int(raw.rPR)) + " SPE " + std::to_string(int(raw.rSP)) + " FOR " + std::to_string(int(raw.rFR)) + " GTO " + std::to_string(1) + "\n";
    write(command);
    unsigned long bytes_read = read(false);
    command_.rPR = raw.rPR;
    command_.rSP = raw.rSP;
    command_.rFR = raw.rFR;
}

void Robotiq2FGripperAPI::getPosition(double *pos)
{
    std::string command = "GET POS\n";
    unsigned long data_sent = getData(command);
    if (data_sent > 0)
    {
        *pos = (double)status_.gPO/pos_to_ticks_ + pos_offset_;
    }
}

void Robotiq2FGripperAPI::getPositionCmd(double *pos)
{
    std::string command = "GET PRE\n";
    unsigned long data_sent = getData(command);
    if (data_sent > 0)
    {
        *pos = (double) status_.gPR / pos_to_ticks_ + pos_offset_;
    }
}

void Robotiq2FGripperAPI::getCurrent(double *cur)
{
    ;
}

void Robotiq2FGripperAPI::getSid(int *sid_p)
{
    std::string command = "GET SID\n";
    unsigned long data_sent = getData(command);
    if (data_sent > 0)
    {
        *sid_p = sid;
    }

}

void Robotiq2FGripperAPI::getGripperStatus(InitializationMode *gACT, ActionMode *gGTO, MotionStatus *gSTA)
{
    std::string command_1 = "GET STA\n";
    unsigned long data_sent = getData(command_1);
    if (data_sent > 0)
    {
        *gSTA = (MotionStatus)status_.gSTA;
    }
    std::string command_2 = "GET ACT\n";
    data_sent = getData(command_2);
    if (data_sent > 0)
    {
        *gACT = (InitializationMode)status_.gACT;
    }
    std::string command_3 = "GET GTO\n";
    data_sent = getData(command_3);
    if (data_sent > 0)
    {
        *gGTO = (ActionMode)status_.gGTO;
    }
}

void Robotiq2FGripperAPI::getFaultStatus(FaultStatus *gFLT)
{
    std::string command = "GET FLT\n";
    unsigned long data_sent = getData(command);
    if (data_sent > 0)
    {
        *gFLT = (FaultStatus)status_.gFLT;
    }
}

void Robotiq2FGripperAPI::getRaw(Robotiq2FGripperClientBase::GripperInput *raw)
{
    std::vector<std::string> commandList = {"GET ACT\n", "GET GTO\n", "GET POS\n", "GET STA\n", "GET PRE\n"};

    for (const auto& command : commandList) {
        getData(command);
    }

    *raw = status_;
}

void Robotiq2FGripperAPI::getCommandPos(double *pos)
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

unsigned long Robotiq2FGripperAPI::getData(std::string request)
{
    bool sent = write(request);
    unsigned long bytes_read = read();
    return bytes_read;
}

bool Robotiq2FGripperAPI::decode(std::string &input)
{
    size_t spacePos = input.find(' ');

    std::string valueStr = input.substr(spacePos + 1);
    std::string variableName = input.substr(0, spacePos);

    if (valueStr.front() == '[') {
        // Remove the first and second last characters (brackets)
        valueStr = valueStr.substr(1, valueStr.size() - 3);
        }

    if (variableName == "ack"){
        return true;
    }

    // Convert the value string to an integer
    if(valueStr.empty())
    {
        return false;
    }

    int value = std::stoi(valueStr);

    // Convert the integer to uint8_t, assuming it fits within the range
    auto valueUint8 = static_cast<uint8_t>(value);

    auto it = statusMap.find(variableName);

    if (it != statusMap.end()) {
        status_.*(it->second) = valueUint8;

        return true;
    }

    if (valueStr == "SID")
    {
        sid = valueUint8;
        return true;
    }
    return false;
}

unsigned long Robotiq2FGripperAPI::read(bool if_decode)
{
    std::string response = base_->readInputs();
    if (!response.empty() && if_decode)
    {
        decode(response);
        return response.size();
    }
    else
    {
        return 0;
    }

}

bool Robotiq2FGripperAPI::write(std::string &command)
{
    bool result = base_->writeOutputs(command);
    return result;
}
