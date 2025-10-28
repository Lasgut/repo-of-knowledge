#include "UdpServer.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UdpServer::UdpServer(int port) : port(port), running(false) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
}

UdpServer::~UdpServer() {
    stop();
}

void UdpServer::start() {
    running = true;
    std::cout << "UDP server listening on port " << port << "..." << std::endl;

    char buffer[1024];
    socklen_t len = sizeof(cliaddr);

    while (running) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, MSG_WAITALL,
                         (struct sockaddr *)&cliaddr, &len);
        if (n < 0) {
            perror("recvfrom error");
            continue;
        }

        buffer[n] = '\0';
        std::string msg(buffer);
        handleClient(msg, cliaddr, len);
    }
}

void UdpServer::handleClient(const std::string &message, const sockaddr_in &clientAddr, socklen_t addrLen) {
    std::cout << "Received from client: " << message << std::endl;

    std::string response = "Server echo: " + message;
    sendto(sockfd, response.c_str(), response.size(), MSG_CONFIRM,
           (const struct sockaddr *)&clientAddr, addrLen);
}

void UdpServer::stop() {
    if (running) {
        running = false;
        close(sockfd);
        std::cout << "Server stopped." << std::endl;
    }
}
