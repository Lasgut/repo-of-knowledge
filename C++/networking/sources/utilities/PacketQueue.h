#pragma once

#include <mutex>
#include <queue>
#include <condition_variable>

#include "Buffer.h"
#include "SocketAddress.h"
        
using Packet = std::pair<Buffer, SocketAddress>;

class PacketQueue
{
public:
    void push(Buffer&& buffer, SocketAddress&& sockAddr);
    bool pop(Buffer& buffer, SocketAddress& sockAddr);

    void setMaxQueueMemoryUsage(int maxBytes) noexcept { maxQueueMemoryUsage_ = maxBytes; }

private:
    std::queue<Packet> queue_;
    std::mutex         queueLock_;

    int maxQueueMemoryUsage_{1000}; // size in bytes
    int currentQueueMemoryUsage_{0};
};