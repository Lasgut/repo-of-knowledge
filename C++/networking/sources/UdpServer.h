#pragma once
#include <string>
#include <netinet/in.h>

class UdpServer {
public:
    UdpServer(int port);
    ~UdpServer();

    void start();
    void stop();

private:
    int sockfd;
    int port;
    bool running;
    struct sockaddr_in servaddr, cliaddr;

    void handleClient(const std::string &message, const sockaddr_in &clientAddr, socklen_t addrLen);
};
