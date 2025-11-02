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

private:
    std::queue<Packet> queue_;
    std::mutex         queueLock_;
};