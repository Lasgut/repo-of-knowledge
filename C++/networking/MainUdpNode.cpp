
#include "UdpNode.h"
#include <thread>
#include <chrono>
#include <iostream>

const Buffer
nodeOnReceiveCallback(const Buffer& buffer, const SocketAddress& clientAddr)
{
    std::cout << "RECEIVE: " << std::endl;
    std::cout << "   Receive from: " << (clientAddr.getIpAddress().has_value() ? clientAddr.getIpAddress().value() : "<unknown>") << ":" << clientAddr.getPort() << std::endl;
    std::cout << "   " << std::string(buffer.getConstPtr(), buffer.size()) << std::endl;
    return {};
}


int main(int argc, char* argv[]) 
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <listen_port> <receive_port>" << std::endl;
        return 1;
    }

    int listenPort = std::stoi(argv[1]);
    int receivePort = std::stoi(argv[2]);
    
    if (listenPort < 1024 || listenPort > 65535 || 
        receivePort < 1024 || receivePort > 65535) {
        std::cerr << "Ports must be between 1024 and 65535" << std::endl;
        return 1;
    }

    UdpNode node(listenPort);
    node.setName("node");
    node.setBufferSize(1024);
    //node.setSocketBufferSize(1);
    node.setNoBlocking(true);
    node.setNoBlockingSleepTime(1);
    node.setClientHandlerSleepTime(1);
    node.setOnReceive(nodeOnReceiveCallback);
    node.setMaxQueueMemoryUsage(24*10);
    node.startListening();

    node.setReceivingAddress("127.0.0.1", receivePort);


    std::cout << "Enter message to send:\n";
    while (true)
    {
        std::string message;
        std::getline(std::cin, message);

        std::cout << "SEND: " << std::endl;
        std::cout << "   Send to: " << node.getReceivingAddress().getIpAddress().value_or("<unknown>") << ":" << node.getReceivingAddress().getPort() << std::endl;
        std::cout << "   " << message << std::endl;

        Buffer buffer(message.size());
        std::memcpy(buffer.getPtr(), message.c_str(), message.size());
        node.sendData(buffer);
    }
}