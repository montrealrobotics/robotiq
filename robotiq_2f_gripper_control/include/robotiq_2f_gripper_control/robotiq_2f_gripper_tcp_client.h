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

#ifndef ROBOTIQ_2F_GRIPPER_TCP_CLIENT_H
#define ROBOTIQ_2F_GRIPPER_TCP_CLIENT_H

#include <robotiq_2f_gripper_control/robotiq_2f_gripper_client_base.h>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


namespace robotiq_2f_gripper_control
{

/**
 * \brief This class provides a client for the TCP object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class Robotiq2FGripperTcpClient : public Robotiq2FGripperClientBase
{
public:
  /**
   * \brief Constructs a control interface to a 2F Robotiq gripper.
   *
   */
  Robotiq2FGripperTcpClient();

  bool connectToServer(const std::string& ipAddress, int port);

  /**
   * \brief Write the given set of control flags to the memory of the gripper
   *
   * @param[in] output The set of output-register values to write to the gripper
   */
  bool writeOutputs(const std::string& output);

  /**
   * \brief Reads set of input-register values from the gripper.
   * \return The gripper input registers as read from the controller IOMap
   */
  std::string readInputs() const;

  void closeConnection();


  int socketFD;

  ~Robotiq2FGripperTcpClient();

private:

};

}

#endif
