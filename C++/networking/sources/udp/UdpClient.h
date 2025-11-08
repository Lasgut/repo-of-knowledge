#pragma once
#include <string>
#include <netinet/in.h>

#include "UdpSocket.h"
#include "Buffer.h"

class UdpClient 
{
    public:
        UdpClient(const std::string ipAddress, int port);
        UdpClient(UdpClient&&)            noexcept;
        UdpClient& operator=(UdpClient&&) noexcept;
        UdpClient(const UdpClient&)            = delete;
        UdpClient& operator=(const UdpClient&) = delete;

        void sendData(const Buffer& buffer);
        void sendData(const Buffer& buffer, const SocketAddress &destAddr);
        Buffer receiveData();

        void setReceiverAddress(const std::string ipAddress, int port);

        std::optional<std::string> getReceiverAddress() const;
        int                        getReceiverPort()    const;

        const SocketAddress& getAddress() const noexcept { return udpSocket_.getAddress(); }

    private:
        SocketAddress receiverAddress_{};
        size_t        bufferSize_{1024};

        UdpSocket udpSocket_;
};
