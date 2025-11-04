#include "PacketQueue.h"
#include <iostream>

void 
PacketQueue::push(Buffer&& buffer, SocketAddress&& sockAddr)
{
    size_t packetSize = buffer.size();
    std::lock_guard<std::mutex> lock(queueLock_);
    if (currentQueueMemoryUsage_ + packetSize > maxQueueMemoryUsage_) // using more than max memory for queue
    {
        std::cerr << "PacketQueue overflow, dropping packet!" << std::endl;
        return;
    }
    queue_.emplace(std::move(buffer), std::move(sockAddr));
    currentQueueMemoryUsage_ += packetSize;
}


bool
PacketQueue::pop(Buffer& buffer, SocketAddress& sockAddr)
{
    std::lock_guard<std::mutex> lock(queueLock_);
    if (queue_.empty())
        return false;
    auto outPacket = std::move(queue_.front());
    queue_.pop();
    buffer   = std::move(outPacket.first);
    sockAddr = std::move(outPacket.second);
    currentQueueMemoryUsage_ -= buffer.size();
    return true;
}
