#pragma once
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <netinet/in.h>
#include <optional>
#include <arpa/inet.h>
#include <fcntl.h>

class SocketAddress 
{
    public:
        SocketAddress();
        SocketAddress(const int port);
        SocketAddress(const std::string& ipAddress);
        SocketAddress(const std::string& ipAddress, const int port);
        ~SocketAddress();
        SocketAddress(SocketAddress&& other) noexcept;             // move constructor
        SocketAddress(const SocketAddress&)              = delete; // no copy
        SocketAddress& operator=(const SocketAddress&)   = delete; // no copy assign
        SocketAddress&& operator=(const SocketAddress&&) = delete; // no move assign

        // Accessors
        int                        getPort()             const noexcept { return ntohs(socketAddress_.sin_port); }
        std::optional<std::string> getIpAddress()        const noexcept { return socketAddress_.sin_addr.s_addr ? std::optional<std::string>(inet_ntoa(socketAddress_.sin_addr)) : std::nullopt; }
        sockaddr*                  getSockAddrPtr()            noexcept { return (sockaddr*)&socketAddress_; }
        const sockaddr*            getConstSockAddrPtr() const noexcept { return (const sockaddr*)&socketAddress_; }
        unsigned int               getSize()             const noexcept { return sizeof(socketAddress_); }

        // Optional: operators for convenience
        SocketAddress& operator=(SocketAddress&& other) noexcept;

    private:
        void createSocketAddressIn(const std::string& ipAddress, const int port);

        sockaddr_in socketAddress_;
};