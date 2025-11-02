#include "UdpSocket.h"
#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>


// OS assignes random available port.
UdpSocket::UdpSocket()
{
    initialize();
}


// User specified port.
UdpSocket::UdpSocket(int port)
    : socketAddress_(port)
{
    initialize();
}


// User specified IP address and port.
UdpSocket::UdpSocket(const std::string& ipAddress, int port)
    : socketAddress_(ipAddress, port)
{
    initialize();
}


UdpSocket::~UdpSocket()
{
    close(socketFileDescriptor_);
}


void 
UdpSocket::sendData(const Buffer& buffer, const SocketAddress& destAddr)
{
    sendto(socketFileDescriptor_, buffer.getConstPtr(), buffer.size(), MSG_CONFIRM,
           destAddr.getConstSockAddrPtr(), destAddr.getSize());
}


int
UdpSocket::receiveData(Buffer& buffer, SocketAddress* srcAddr)
{
    // if srcAddr == nullptr
    SocketAddress  tmpAddr{};
    SocketAddress* addrPtr = srcAddr ? srcAddr : &tmpAddr;
    auto           addrLen = addrPtr->getSize();

    int n = recvfrom(
        socketFileDescriptor_,
        buffer.getPtr(), 
        buffer.size(),
        0, // No special flags
        addrPtr->getSockAddrPtr(), 
        &addrLen
    );

    return n;
}


void 
UdpSocket::bindSocket()
{
    // Associates the socket with the specified IP and port.
    if (bind(socketFileDescriptor_, (const struct sockaddr *)&socketAddress_, sizeof(socketAddress_)) < 0) 
    {
        perror("Bind failed");
        close(socketFileDescriptor_); // Close socket on failure
        exit(EXIT_FAILURE);
    }
}


void 
UdpSocket::closeSocket()
{
    close(socketFileDescriptor_);
}


void 
UdpSocket::setNoBlocking(bool enable)
{
    int flags = fcntl(socketFileDescriptor_, F_GETFL, 0);
    if (flags == -1) flags = 0;

    if (enable)
        fcntl(socketFileDescriptor_, F_SETFL, flags | O_NONBLOCK);
    else
        fcntl(socketFileDescriptor_, F_SETFL, flags & ~O_NONBLOCK);
}


void 
UdpSocket::initialize()
{
    createSocket();
    bindSocket();
}


void UdpSocket::createSocket()
{
    // AF_INET: Specifies IPv4. 
    // SOCK_DGRAM: Specifies datagram socket (UDP).
    // 0: Default protocol (UDP for SOCK_DGRAM).
    socketFileDescriptor_ = socket(AF_INET, SOCK_DGRAM, 0); 
    if (socketFileDescriptor_ < 0) 
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
}