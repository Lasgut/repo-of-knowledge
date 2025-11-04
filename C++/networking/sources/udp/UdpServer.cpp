#include "UdpServer.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>


UdpServer::UdpServer(int port)
    : udpSocket_(port)
{
}


UdpServer::UdpServer(const std::string ipAddress, int port)
    : udpSocket_(ipAddress, port)
{
}


UdpServer::~UdpServer() 
{
    stop();
}


void 
UdpServer::start() 
{
    auto listenThreadName = createThreadName("Lisn");
    //auto listenThreadName = std::string("UDP_SERVER_THREAD: "+name_+" listen");
    listenThread_ = std::thread([&]() { 
        pthread_setname_np(pthread_self(), listenThreadName.c_str());
        listenLoop(); 
    });
    while (!running_) {} // hold until running  is true
    auto clientQueueThreadName = createThreadName("CliHndlr");
    clientQueueThread_ = std::thread([&]() {
        pthread_setname_np(pthread_self(), clientQueueThreadName.c_str());
        clientHandlerLoop(); 
    });
}


void 
UdpServer::listen()
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
UdpServer::listenLoop()
{
    auto ipAddr = udpSocket_.getAddress().getIpAddress();
    auto port   = udpSocket_.getAddress().getPort();
    std::cout << "UDP server listening on " << (ipAddr.has_value() ? ipAddr.value() : "<any>") << ":" << port << std::endl;
    running_ = true;
    while (running_) 
    {
        listen();
    }
}


void 
UdpServer::clientHandlerLoop()
{
    while (running_)
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
UdpServer::handleClient(const Buffer& buffer, const SocketAddress& clientAddr)
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
UdpServer::createThreadName(const std::string &name)
{
    auto threadName = name_+name;
    if (threadName.size() > 15)
    {
        std::cout << "Warning: Thread name '" << threadName << "' is too long, truncating to 15 characters." << std::endl;
        threadName = threadName.substr(0, 15); // pthread_setname_np limit
    }
    return threadName;
}


void 
UdpServer::stop() 
{
    if (running_)
    {
        std::cout << "Server stopped." << std::endl;
    }
    running_ = false;
    udpSocket_.closeSocket();
    if (listenThread_.joinable())        listenThread_.join();
    if (clientQueueThread_.joinable())   clientQueueThread_.join();
}


/**
 * @brief Sets a callback function to be called when the server receives data.
 * 
 * The callback function is triggered every time a datagram arrives.
 * It receives the incoming data buffer and the sender's address.
 * The callback can optionally return a `Buffer` to send a response back to the sender.
 *
 * @param callback A function with signature `Buffer(const Buffer&, const SocketAddress&)`
 *                 that will be called for every received message.
 *
 * @note Make sure your callback is thread-safe if the server is running in a separate thread.
 */
void 
UdpServer::setOnReceive(std::function<Buffer(const Buffer&, const SocketAddress&)> callback)
{
    onReceiveCallback_ = callback;
}


/**
 * @brief Sets whether the server socket should operate in non-blocking mode.
 * 
 * When `noBlocking` is set to `false`:
 * - The underlying `recvfrom()` call will block the thread until a packet arrives.
 * - This can cause the program to hang if the server is running in a separate thread.
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
UdpServer::setNoBlocking(bool noBlocking)
{
    noBlocking_ = noBlocking; 
    udpSocket_.setNoBlocking(noBlocking);
}


/**
 * @brief Sets the sleep time (in milliseconds) for the server loop when using non-blocking mode.
 * 
 * When the server socket is in non-blocking mode (`noBlocking == true`), the receive loop
 * continuously checks for incoming packets. Without a short sleep, this can lead to
 * 100% CPU usage. This function allows you to configure how long the thread should sleep
 * between each poll of the socket.
 *
 * @param milliSeconds The number of milliseconds to sleep between receive attempts.
 */
void 
UdpServer::setNoBlockingSleepTime(int nanoSeconds)
{
    noBlockingSleepTimeNs_ = nanoSeconds;
}


void 
UdpServer::setClientHandlerSleepTime(int nanoSeconds)
{
    clientHandlerSleepTimeNs_ = nanoSeconds;
}


void 
UdpServer::setName(std::string name)
{
    name_ = name;
}
