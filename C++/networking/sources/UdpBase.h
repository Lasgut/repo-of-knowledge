#pragma once
#include <string>
#include <netinet/in.h>
#include <unistd.h>
#include <optional>

class UdpBase
{
    public:
        UdpBase();
        UdpBase(int port);
        UdpBase(const std::string &ipAddress, int port);
        ~UdpBase();

        void setBufferSize(size_t size); 

    protected:
        void createSocket();

        int                         socketFileDescriptor_{0};
        size_t                      bufferSize_{1024};         // Buffer size in bytes
        int                         port_{0};                  // OS assigns random available port if port == 0
        std::optional<std::string>  ipAddress_;
};