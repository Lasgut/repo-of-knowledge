#pragma once
#include <string>
#include <netinet/in.h>
#include <optional>
#include "UdpBase.h"

class UdpServer 
    : UdpBase
{
    public:
        UdpServer();
        UdpServer(int port);
        UdpServer(const std::string &ipAddress, int port);
        ~UdpServer();

        void start();
        void stop();
        
        void sendDataToClient(
            const char* data,
            size_t dataSize,
            const sockaddr_in &clientAddr, 
            socklen_t addrLen
        );

    private:
        void handleClient(const std::string &message, const sockaddr_in &clientAddr, socklen_t addrLen);

        void initialize();
        void createServerAddress();
        void bindSocket();

        bool   running_{false};

        sockaddr_in serverAddress_{};
        sockaddr_in clientAddress_{};
};
