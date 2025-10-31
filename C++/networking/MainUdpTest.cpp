#include "sources/UdpServer.h"
#include "sources/UdpClient.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <arpa/inet.h>

Buffer 
serverOnReceiveCallback(const Buffer& buffer, const SocketAddress& clientAddr)
{
    std::cout << "SERVER: " << std::endl;
    std::cout << "   Receive from: " << (clientAddr.getIpAddress().has_value() ? clientAddr.getIpAddress().value() : "<unknown>") << ":" << clientAddr.getPort() << std::endl;
    std::cout << "   " << std::string(buffer.getConstPtr(), buffer.size()) << std::endl;
    return {};
}

int main() 
{
    // Start server in a separate thread
    UdpServer server(8080);
    server.setNoBlocking(true);
    server.setOnReceive(serverOnReceiveCallback);
    server.start();

    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for server to start

    // Create client and communicate
    UdpClient client("127.0.0.1", 8081);
    client.setReceiverAddress("127.0.0.1", 8080);

    std::string message = "Hello from client!";
    auto buffer = Buffer(message.size());
    std::memcpy(buffer.getPtr(), message.c_str(), message.size());
    client.sendData(buffer);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    server.stop();
    return 0;
}
