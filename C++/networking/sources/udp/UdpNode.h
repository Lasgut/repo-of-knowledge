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

class UdpNode
{
    public:
        UdpNode(int port);
        UdpNode(const std::string ipAddress, int port);
        ~UdpNode();

        void startListening();
        void stopListening();

        void sendData(const Buffer& buffer);
        void sendData(const Buffer& buffer, const SocketAddress &destAddr);

        void setOnReceive(std::function<Buffer(const Buffer&, const SocketAddress&)> callback);

        const SocketAddress& getSelfAddress()      const noexcept { return udpSocket_.getAddress(); }
        const SocketAddress& getReceivingAddress() const noexcept { return receiverAddress_; }

        // settings
        void setBufferSize(size_t size) { bufferSize_ = size; }
        void setSocketBufferSize(size_t size) { udpSocket_.setSocketBufferSize(size); }
        void setNoBlocking(bool noBlocking);
        void setNoBlockingSleepTime(int nanoSeconds);
        void setClientHandlerSleepTime(int nanoSeconds);
        void setName(std::string name);
        void setMaxQueueMemoryUsage(int maxBytes) { clientQueue_.setMaxQueueMemoryUsage(maxBytes); }
        void setReceivingAddress(const std::string& ipAddress, const int port);

    private:
        void listen();
        void listenLoop();
        void handleClient(const Buffer& buffer, const SocketAddress& clientAddr);
        void clientHandlerLoop();

        std::string createThreadName(const std::string& name);

        UdpSocket         udpSocket_;
        SocketAddress     receiverAddress_{};
        std::atomic<bool> listening_{false};
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
