#pragma once
#include <string>
#include <netinet/in.h>

#include "UdpSocket.h"
#include "Buffer.h"

class UdpClient 
{
    public:
        UdpClient(const std::string ipAddress, int port);

        void sendData(const Buffer& buffer);
        void sendData(const Buffer& buffer, const SocketAddress &destAddr);
        Buffer receiveData();

        void setReceiverAddress(const std::string ipAddress, int port);

        const SocketAddress& getAddress() const noexcept { return udpSocket_.getAddress(); }

    private:
        SocketAddress receiverAddress_{};
        size_t        bufferSize_{1024};

        UdpSocket udpSocket_;
};
