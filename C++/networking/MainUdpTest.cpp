#include "sources/UdpServer.h"
#include "sources/UdpClient.h"
#include <thread>
#include <chrono>
#include <iostream>

int main() {
    // Start server in a separate thread
    UdpServer server(8080);
    std::thread serverThread([&]() { server.start(); });

    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for server to start

    // Create client and communicate
    UdpClient client("127.0.0.1", 8080);
    client.sendMessage("Hello from client!");
    //std::string reply = client.receiveMessage();
    //std::cout << "Server replied: " << reply << std::endl;

    client.sendMessage("tata");

    std::this_thread::sleep_for(std::chrono::seconds(1));
    server.stop();

    serverThread.join();
    return 0;
}
