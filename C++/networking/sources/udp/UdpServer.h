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
        UdpServer();
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
        void setNoBlockingSleepTime(int milliSeconds); 
        void setName(std::string name);

    private:
        void listen();
        void listenLoop();
        void clientHandlerLoop();
        void handleClient(const Buffer& buffer, const SocketAddress& clientAddr);

        UdpSocket         udpSocket_;
        std::atomic<bool> running_{false};
        bool              noBlocking_{true};
        int               noBlockingSleepTimeMs_{100};
        size_t            bufferSize_{1024};
        std::string       name_;

        std::thread listenThread_;
        std::thread clientQueueThread_;
        PacketQueue clientQueue_;
        std::function<Buffer(const Buffer&, const SocketAddress&)> onReceiveCallback_;
};
