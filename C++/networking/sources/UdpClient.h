#pragma once
#include <string>
#include <netinet/in.h>

class UdpClient {
public:
    UdpClient(const std::string &serverIp, int port);
    ~UdpClient();

    void sendMessage(const std::string &message);
    std::string receiveMessage();

private:
    int sockfd;
    struct sockaddr_in servaddr;
};
