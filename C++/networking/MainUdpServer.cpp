#include "sources/UdpServer.h"
#include <thread>
#include <chrono>
#include <iostream>

const Buffer
serverOnReceiveCallback(const Buffer& buffer, const SocketAddress& clientAddr)
{
    std::cout << "SERVER: " << std::endl;
    std::cout << "   Receive from: " << (clientAddr.getIpAddress().has_value() ? clientAddr.getIpAddress().value() : "<unknown>") << ":" << clientAddr.getPort() << std::endl;
    std::cout << "   " << std::string(buffer.getConstPtr(), buffer.size()) << std::endl;
    
    Buffer responseBuffer(30);
    std::string responseMessage = "Hello from server!";
    std::memcpy(responseBuffer.getPtr(), responseMessage.c_str(), responseMessage.size());

    return responseBuffer;
}

int main() 
{
    UdpServer server(8080);
    server.setNoBlocking(true);
    server.setOnReceive(serverOnReceiveCallback);
    server.start();
    // Main thread waits until user stops server
    std::cout << "Press Enter to stop server..." << std::endl;
    std::cin.get();
    server.stop();
    return 0;
}