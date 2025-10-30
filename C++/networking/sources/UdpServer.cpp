#include "UdpServer.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UdpServer::UdpServer() 
{
    initialize();
}


UdpServer::UdpServer(int port)
    : UdpBase(port)
{
    initialize();
}


UdpServer::UdpServer(const std::string &ipAddress, int port)
    : UdpBase(ipAddress, port)
{
    initialize();
}


UdpServer::~UdpServer() 
{
    stop();
}


void 
UdpServer::start() 
{
    running_ = true;
    std::cout << "UDP server listening on " << (ipAddress_.has_value() ? ipAddress_.value() : "<any>") << ":" << port_ << std::endl;

    char buffer[bufferSize_];               // Allocates a buffer of bufferSize_ bytes to store incoming data.
    socklen_t len = sizeof(clientAddress_); // Length of client address

    while (running_) 
    {
        int n = recvfrom(
            socketFileDescriptor_,
            buffer, 
            sizeof(buffer),
            0, // No special flags
            (struct sockaddr *)&clientAddress_, 
            &len
        );

        if (n < 0) 
        {
            perror("recvfrom error");
            exit(EXIT_FAILURE);
        }

        handleClient(buffer, clientAddress_, len);
        memset(buffer, 0, sizeof(buffer)); // Clear buffer for next message
    }
}


void 
UdpServer::handleClient(const std::string &message, const sockaddr_in &clientAddr, socklen_t addrLen) 
{
    // Convert client address to human-readable form
    char ipStr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(clientAddr.sin_addr), ipStr, INET_ADDRSTRLEN);

    // Convert port to human-readable form
    uint16_t port = ntohs(clientAddr.sin_port);

    std::cout << "Received from client: " << ipStr << ":" << port << "\n   " << message << std::endl;
}


void 
UdpServer::initialize()
{
    createSocket();
    createServerAddress();
    bindSocket();
}


void UdpServer::createServerAddress()
{
    memset(&serverAddress_, 0, sizeof(serverAddress_));   // Clear server address structure
    serverAddress_.sin_family = AF_INET;     // IPv4
    serverAddress_.sin_port = htons(port_);  // Convert port number to network byte order
    if (ipAddress_.has_value())
    {
        // Binding to a specific IP address
        inet_pton(AF_INET, ipAddress_->c_str(), &serverAddress_.sin_addr); 
    }
    else
    {
        // Accept connections from any address
        serverAddress_.sin_addr.s_addr = INADDR_ANY;   
    }
}


void UdpServer::bindSocket()
{
    // Associates the socket with the specified IP and port.
    if (bind(socketFileDescriptor_, (const struct sockaddr *)&serverAddress_, sizeof(serverAddress_)) < 0) 
    {
        perror("Bind failed");
        close(socketFileDescriptor_); // Close socket on failure
        exit(EXIT_FAILURE);
    }
}


void 
UdpServer::stop() 
{
    if (running_) 
    {
        running_ = false;
        close(socketFileDescriptor_);
        std::cout << "Server stopped." << std::endl;
    }
}


void 
UdpServer::sendDataToClient(const char *data, size_t dataSize, const sockaddr_in &clientAddr, socklen_t addrLen)
{
    sendto(socketFileDescriptor_, data, dataSize, MSG_CONFIRM,
           (const struct sockaddr *)&clientAddr, addrLen);
}
