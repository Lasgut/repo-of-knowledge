#include "sources/UdpClient.h"
#include <thread>
#include <chrono>
#include <iostream>

int main() 
{
    UdpClient client("127.0.0.1", 8081);
    client.setReceiverAddress("127.0.0.1", 8080);

    while (true)
    {
        std::string message;
        std::cout << "Enter message to send: ";
        std::getline(std::cin, message);

        std::cout << "CLIENT: " << std::endl;
        std::cout << "   Send to: " << client.getAddress().getIpAddress().value_or("<unknown>") << ":" << client.getAddress().getPort() << std::endl;
        std::cout << "   " << message << std::endl;

        Buffer buffer(message.size());
        std::memcpy(buffer.getPtr(), message.c_str(), message.size());
        client.sendData(buffer);

        auto responseBuffer = client.receiveData();

        std::cout << "CLIENT: Received response: " << std::string(responseBuffer.getPtr(), responseBuffer.size()) << std::endl;
    }
}