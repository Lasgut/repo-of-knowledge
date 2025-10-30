#include "UdpClient.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UdpClient::UdpClient(const std::string &serverIp, int port) 
    : UdpBase(port)
{
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);

    if (inet_pton(AF_INET, serverIp.c_str(), &servaddr.sin_addr) <= 0) {
        perror("Invalid address");
        close(socketFileDescriptor_);
        exit(EXIT_FAILURE);
    }
}


void 
UdpClient::sendMessage(const std::string &message) 
{
    sendto(
        socketFileDescriptor_, 
        message.c_str(), 
        message.size(), 
        0,
        (const struct sockaddr *)&servaddr, 
        sizeof(servaddr)
    );
    std::cout << "Message sent: " << message << std::endl;
}


std::string 
UdpClient::receiveMessage() 
{
    char buffer[bufferSize_];
    socklen_t len = sizeof(servaddr);
    int n = recvfrom(
        socketFileDescriptor_, 
        buffer, 
        sizeof(buffer), 
        0,
        (struct sockaddr *)&servaddr, 
        &len
    );

    return std::string(buffer);
}
