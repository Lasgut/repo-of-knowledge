#include "UdpClient.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UdpClient::UdpClient(const std::string ipAddress, int port) 
    : udpSocket_(ipAddress, port)
{
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
