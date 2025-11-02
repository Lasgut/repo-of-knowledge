#include "PacketQueue.h"


void 
PacketQueue::push(Buffer&& buffer, SocketAddress&& sockAddr)
{
    std::lock_guard<std::mutex> lock(queueLock_);
    queue_.emplace(std::move(buffer), std::move(sockAddr));
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
    return true;
}
