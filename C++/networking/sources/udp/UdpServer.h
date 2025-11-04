#pragma once
#include <string>
#include <netinet/in.h>
#include <optional>
#include <functional>
#include <thread>
#include <atomic>

#include "UdpSocket.h"
#include "Buffer.h"
#include "PacketQueue.h"

class UdpServer 
{
    public:
        UdpServer(int port);
        UdpServer(const std::string ipAddress, int port);
        ~UdpServer();

        void start();
        void stop();

        void setOnReceive(std::function<Buffer(const Buffer&, const SocketAddress&)> callback);

        const SocketAddress& getAddress() const noexcept { return udpSocket_.getAddress(); }

        // settings
        void setBufferSize(size_t size) { bufferSize_ = size; }
        void setSocketBufferSize(size_t size) { udpSocket_.setSocketBufferSize(size); }
        void setNoBlocking(bool noBlocking);
        void setNoBlockingSleepTime(int nanoSeconds);
        void setClientHandlerSleepTime(int nanoSeconds);
        void setName(std::string name);
        void setMaxQueueMemoryUsage(int maxBytes) { clientQueue_.setMaxQueueMemoryUsage(maxBytes); }

    private:
        void listen();
        void listenLoop();
        void clientHandlerLoop();
        void handleClient(const Buffer& buffer, const SocketAddress& clientAddr);

        std::string createThreadName(const std::string& name);

        UdpSocket         udpSocket_;
        std::atomic<bool> running_{false};
        bool              noBlocking_{true};
        int               noBlockingSleepTimeNs_{1000000};
        int               clientHandlerSleepTimeNs_{1000000};
        size_t            bufferSize_{1024};
        std::string       name_{"unnamed"};

        std::thread listenThread_;
        std::thread clientQueueThread_;
        PacketQueue clientQueue_;
        std::function<Buffer(const Buffer&, const SocketAddress&)> onReceiveCallback_;
};
