#include "UdpClient.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UdpClient::UdpClient(const std::string &serverIp, int port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);

    if (inet_pton(AF_INET, serverIp.c_str(), &servaddr.sin_addr) <= 0) {
        perror("Invalid address");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
}

UdpClient::~UdpClient() {
    close(sockfd);
}

void UdpClient::sendMessage(const std::string &message) {
    sendto(sockfd, message.c_str(), message.size(), MSG_CONFIRM,
           (const struct sockaddr *)&servaddr, sizeof(servaddr));
    std::cout << "Message sent: " << message << std::endl;
}

std::string UdpClient::receiveMessage() {
    char buffer[1024];
    socklen_t len = sizeof(servaddr);
    int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, MSG_WAITALL,
                     (struct sockaddr *)&servaddr, &len);
    buffer[n] = '\0';
    return std::string(buffer);
}
