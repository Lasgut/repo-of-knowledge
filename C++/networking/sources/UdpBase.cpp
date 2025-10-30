#include "UdpBase.h"

UdpBase::UdpBase()
{
    createSocket();
}


UdpBase::UdpBase(int port)
    : port_(port)
{
    createSocket();
}


UdpBase::UdpBase(const std::string & ipAddress, int port)
    : ipAddress_(ipAddress)
    , port_(port)
{
    createSocket();
}


UdpBase::~UdpBase()
{
    close(socketFileDescriptor_);
}


void 
UdpBase::setBufferSize(size_t size)
{
    bufferSize_ = size;
    if (setsockopt(socketFileDescriptor_, SOL_SOCKET, SO_RCVBUF, &bufferSize_, sizeof(bufferSize_)) < 0) 
    {
        perror("Failed to set receive buffer size");
    }
}


void 
UdpBase::createSocket()
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