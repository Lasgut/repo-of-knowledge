#pragma once
#include <string>
#include <netinet/in.h>
#include <unistd.h>
#include <optional>
#include <atomic>

#include "Buffer.h"
#include "SocketAddress.h"

class UdpSocket
{
    public:
        UdpSocket();
        UdpSocket(int port);
        UdpSocket(const std::string& ipAddress, int port);
        ~UdpSocket();
        UdpSocket(UdpSocket&&)            noexcept;
        UdpSocket& operator=(UdpSocket&&) noexcept;
        UdpSocket(const UdpSocket&)            = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        void sendData   (const Buffer& buffer, const SocketAddress &destAddr);
        int  receiveData(Buffer& buffer, SocketAddress* srcAddr = nullptr);

        void bindSocket();
        void closeSocket();

        void setNoBlocking(bool enable);
        void setSocketBufferSize(size_t size);

        const SocketAddress& getAddress() const noexcept { return socketAddress_; }

    protected:
        void initialize();
        void createSocket();

        std::atomic<int> socketFileDescriptor_{0};
        SocketAddress    socketAddress_{};
};