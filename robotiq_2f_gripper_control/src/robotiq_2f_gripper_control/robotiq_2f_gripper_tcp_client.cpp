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

#include "robotiq_2f_gripper_control/robotiq_2f_gripper_tcp_client.h"

// See Robotiq's documentation for the register mapping

// An effort to keep the lines less than 100 char long
namespace robotiq_2f_gripper_control
{
Robotiq2FGripperTcpClient::Robotiq2FGripperTcpClient()
        : socketFD(-1) {}

bool Robotiq2FGripperTcpClient::connectToServer(const std::string& ipAddress, int port) {
     // Create a TCP socket
     socketFD = socket(AF_INET, SOCK_STREAM, 0);
     if (socketFD == -1) {
         std::cerr << "Error creating socket." << std::endl;
         return false;
     }

     // Define the server address and port
     struct sockaddr_in serverAddress{};
     serverAddress.sin_family = AF_INET;
     serverAddress.sin_port = htons(port);
     inet_pton(AF_INET, ipAddress.c_str(), &serverAddress.sin_addr);

     // Connect to the server
     if (connect(socketFD, reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress)) == -1) {
         std::cerr << "Error connecting to server." << std::endl;
         close(socketFD);
         return false;
     }

     return true;
 }

/*
  See support.robotiq.com -> manual for the register output meanings
*/
bool Robotiq2FGripperTcpClient::writeOutputs(const std::string& output)
{
    const char *utf8Data = output.c_str();

    // Get the length of the UTF-8 string
    size_t dataSize = strlen(utf8Data);

    // Send the data over the socket
    ssize_t bytesSent = send(socketFD, utf8Data, dataSize, 0);

    // Check if sending was successful
    if (bytesSent == -1) {
        std::cerr << "Error sending data over socket." << std::endl;
        return false;
    }

//    for (int i = 0; i < bytesSent; ++i) {
//        std::cout << output[i];
//    }
//    std::cout << std::endl;
    return true;
}

std::string Robotiq2FGripperTcpClient::readInputs() const
{
    // Buffer to store received data
    const int bufferSize = 1024;
    char buffer[bufferSize];

    // Receive data from the socket
    ssize_t bytesRead = recv(socketFD, buffer, bufferSize - 1, 0);

    if (bytesRead == -1) {
        std::cerr << "Error receiving data from server." << std::endl;
        return "";
    }

    // Null-terminate the received data
    buffer[bytesRead] = '\0';

//    for (int i = 0; i < bytesRead; ++i) {
//        std::cout << buffer[i];
//    }
//    std::cout << std::endl;

    return {buffer};
}

void Robotiq2FGripperTcpClient::closeConnection() {
    if (socketFD != -1) {
        close(socketFD);
        socketFD = -1;
    }
}

Robotiq2FGripperTcpClient::~Robotiq2FGripperTcpClient() {
    closeConnection();
}

} // end of robotiq_2f_gripper_control namespace
