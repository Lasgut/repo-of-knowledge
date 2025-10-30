#pragma once
#include <string>
#include <netinet/in.h>

#include "UdpBase.h"

class UdpClient 
    : UdpBase
{
    public:
        UdpClient(const std::string &serverIp, int port);

        void sendMessage(const std::string &message);
        std::string receiveMessage();

    private:
        struct sockaddr_in servaddr;
};
