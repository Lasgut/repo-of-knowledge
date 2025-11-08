#include "UdpNode.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>


UdpNode::UdpNode(int port)
    : udpSocket_(port)
{
}


UdpNode::UdpNode(const std::string ipAddress, int port)
    : udpSocket_(ipAddress, port)
{
}


UdpNode::~UdpNode() 
{
    stopListening();
}


void 
UdpNode::startListening() 
{
    auto listenThreadName = createThreadName("Lisn");
    listenThread_ = std::thread([this, listenThreadName]() { 
        pthread_setname_np(pthread_self(), listenThreadName.c_str());
        listenLoop(); 
    });
    while (!listening_) {} // hold until listening_ is true
    auto clientQueueThreadName = createThreadName("CliHndlr");
    clientQueueThread_ = std::thread([this, clientQueueThreadName]() {
        pthread_setname_np(pthread_self(), clientQueueThreadName.c_str());
        clientHandlerLoop(); 
    });
}


void 
UdpNode::stopListening() 
{
    if (listening_)
    {
        std::cout << "Node stopped listening." << std::endl;
    }
    listening_ = false;
    udpSocket_.closeSocket();
    if (listenThread_.joinable())        listenThread_.join();
    if (clientQueueThread_.joinable())   clientQueueThread_.join();
}


void 
UdpNode::sendData(const Buffer& buffer) 
{

    udpSocket_.sendData(buffer, receiverAddress_);
}


void 
UdpNode::sendData(const Buffer &buffer, const SocketAddress &destAddr)
{
    udpSocket_.sendData(buffer, destAddr);
}


void 
UdpNode::listen()
{
    Buffer        buffer(bufferSize_); // Allocates a buffer of bufferSize_ bytes to store incoming data.
    SocketAddress sourceAddress{};
    auto n = udpSocket_.receiveData(buffer, &sourceAddress);
    if (n > 0)
    {
        // Buffer resizedBuffer(n);
        // std::memcpy(resizedBuffer.getPtr(), buffer.getConstPtr(), resizedBuffer.size());
        // clientQueue_.push(std::move(resizedBuffer), std::move(sourceAddress));
        buffer.resize(n);
        clientQueue_.push(std::move(buffer), std::move(sourceAddress));
        return;
    }
    if (n < 0 && !noBlocking_)
    {
        perror("Receive failed");
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::nanoseconds(noBlockingSleepTimeNs_));
    }
}


void 
UdpNode::listenLoop()
{
    auto ipAddr = udpSocket_.getAddress().getIpAddress();
    auto port   = udpSocket_.getAddress().getPort();
    std::cout << "UDP node listening on " << (ipAddr.has_value() ? ipAddr.value() : "<any>") << ":" << port << std::endl;
    listening_ = true;
    while (listening_) 
    {
        listen();
    }
}


void 
UdpNode::clientHandlerLoop()
{
    while (listening_)
    {
        Buffer        buffer;
        SocketAddress sourceAddress;
        if (clientQueue_.pop(buffer, sourceAddress))
        {
            handleClient(buffer, sourceAddress);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(clientHandlerSleepTimeNs_));
        }
    }
}


void 
UdpNode::handleClient(const Buffer& buffer, const SocketAddress& clientAddr)
{
    if (onReceiveCallback_)
    {
        auto responseBuffer = onReceiveCallback_(buffer, clientAddr);
        if (responseBuffer.size() > 0)
        {
            udpSocket_.sendData(responseBuffer, clientAddr);
        }
    }
}


std::string
UdpNode::createThreadName(const std::string& name)
{
    auto threadName = name_ + name;
    if (threadName.size() > 15)
    {
        std::cout << "Warning: Thread name '" << threadName << "' is too long, truncating to 15 characters." << std::endl;
        threadName = threadName.substr(0, 15); // pthread_setname_np limit
    }
    return threadName;
}


/**
 * @brief Sets a callback function to be called when the node receives data.
 * 
 * The callback function is triggered every time a datagram arrives.
 * It receives the incoming data buffer and the sender's address.
 * The callback can optionally return a `Buffer` to send a response back to the sender.
 *
 * @param callback A function with signature `Buffer(const Buffer&, const SocketAddress&)`
 *                 that will be called for every received message.
 *
 * @note Make sure your callback is thread-safe if the node is running in a separate thread.
 */
void 
UdpNode::setOnReceive(std::function<Buffer(const Buffer&, const SocketAddress&)> callback)
{
    onReceiveCallback_ = callback;
}


/**
 * @brief Sets whether the node socket should operate in non-blocking mode.
 * 
 * When `noBlocking` is set to `false`:
 * - The underlying `recvfrom()` call will block the thread until a packet arrives.
 * - This can cause the program to hang if the node is running in a separate thread.
 * 
 * When `noBlocking` is set to `true`:
 * - `recvfrom()` will return immediately if no data is available.
 * - You will need to poll in a loop to check for incoming packets.
 * - To avoid 100% CPU usage, a short sleep is introduced between each poll.
 * - Set sleep time using `setNoBlockingSleepTime()`.
 * 
 * @param noBlocking If true, the socket is non-blocking; if false, it is blocking.
 */
void 
UdpNode::setNoBlocking(bool noBlocking)
{
    noBlocking_ = noBlocking; 
    udpSocket_.setNoBlocking(noBlocking);
}


/**
 * @brief Sets the sleep time (in milliseconds) for the node loop when using non-blocking mode.
 * 
 * When the node socket is in non-blocking mode (`noBlocking == true`), the receive loop
 * continuously checks for incoming packets. Without a short sleep, this can lead to
 * 100% CPU usage. This function allows you to configure how long the thread should sleep
 * between each poll of the socket.
 *
 * @param milliSeconds The number of milliseconds to sleep between receive attempts.
 */
void 
UdpNode::setNoBlockingSleepTime(int nanoSeconds)
{
    noBlockingSleepTimeNs_ = nanoSeconds;
}


void 
UdpNode::setClientHandlerSleepTime(int nanoSeconds)
{
    clientHandlerSleepTimeNs_ = nanoSeconds;
}


void 
UdpNode::setName(std::string name)
{
    name_ = name;
}


void 
UdpNode::setReceivingAddress(const std::string &ipAddress, const int port)
{
    receiverAddress_ = SocketAddress(ipAddress, port);
}
