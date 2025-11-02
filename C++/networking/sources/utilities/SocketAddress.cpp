#include "SocketAddress.h"
#include <arpa/inet.h>

SocketAddress::SocketAddress()
{
    createSocketAddressIn("", 0);
}


SocketAddress::SocketAddress(const int port)
{
    createSocketAddressIn("", port);
}


SocketAddress::SocketAddress(const std::string &ipAddress)
{
    createSocketAddressIn(ipAddress, 0);
}


SocketAddress::SocketAddress(const std::string &ipAddress, const int port)
{
    createSocketAddressIn(ipAddress, port);
}


SocketAddress::~SocketAddress()
{
}


SocketAddress::SocketAddress(SocketAddress &&other) noexcept
    : socketAddress_(other.socketAddress_)
{
}


SocketAddress 
&SocketAddress::operator=(SocketAddress &&other) noexcept
{
    if (this != &other)
    {
        socketAddress_ = other.socketAddress_;
    }
    return *this;
}


void 
SocketAddress::createSocketAddressIn(const std::string &ipAddress, const int port)
{
    memset(&socketAddress_, 0, sizeof(socketAddress_)); // Clear IP address structure
    socketAddress_.sin_family = AF_INET;                // IPv4
    socketAddress_.sin_port = htons(port);             // Convert port number to network byte order
    if (ipAddress != "")
    {
        // Binding to a specific IP address
        if (inet_pton(AF_INET, ipAddress.c_str(), &socketAddress_.sin_addr) != 1)
        {
            throw std::invalid_argument("Invalid IP address: " + ipAddress);
        }
    }
    else
    {
        // Accept connections from any address
        socketAddress_.sin_addr.s_addr = INADDR_ANY;   
    }
}
