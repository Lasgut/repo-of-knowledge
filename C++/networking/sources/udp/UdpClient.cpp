#include "UdpClient.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UdpClient::UdpClient(const std::string ipAddress, int port) 
    : udpSocket_(ipAddress, port)
{
}


UdpClient::UdpClient(UdpClient&& other) noexcept
    : receiverAddress_(std::move(other.receiverAddress_))
    , bufferSize_(other.bufferSize_)
    , udpSocket_(std::move(other.udpSocket_))
{
    other.bufferSize_ = 0;
}


UdpClient& UdpClient::operator=(UdpClient&& other) noexcept
{
    if (this != &other)
    {
        receiverAddress_ = std::move(other.receiverAddress_);
        bufferSize_ = other.bufferSize_;
        udpSocket_ = std::move(other.udpSocket_);
        other.bufferSize_ = 0;
    }
    return *this;
}


void 
UdpClient::sendData(const Buffer& buffer) 
{

    udpSocket_.sendData(buffer, receiverAddress_);
}


void 
UdpClient::sendData(const Buffer &buffer, const SocketAddress &destAddr)
{
    udpSocket_.sendData(buffer, destAddr);
}


Buffer
UdpClient::receiveData() 
{
    Buffer buffer(bufferSize_);
    udpSocket_.receiveData(buffer);
    return buffer;
}


void 
UdpClient::setReceiverAddress(const std::string ipAddress, int port)
{
    receiverAddress_ = SocketAddress(ipAddress, port);
}


std::optional<std::string> 
UdpClient::getReceiverAddress() const
{
    return receiverAddress_.getIpAddress();
}


int 
UdpClient::getReceiverPort() const
{
    return receiverAddress_.getPort();
}
