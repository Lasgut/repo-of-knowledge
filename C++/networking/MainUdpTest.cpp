#include <thread>
#include <chrono>
#include <iostream>
#include <arpa/inet.h>

#include "UdpServer.h"
#include "UdpClient.h"
#include "Message.h"
#include "MessageType.h"
#include "Serializer.h"

void 
printBufferHex(const Buffer& buffer)
{
    const char* data = buffer.getConstPtr();
    size_t size = buffer.size();
    std::cout << "   Buffer in HEX:      ";
    for (size_t i = 0; i < size; ++i)
    {
        printf("%02X ", static_cast<unsigned char>(data[i]));
    }
    std::cout << std::endl;
}


template <MessageType MsgType>
void 
printSomeSizes(const Message<MsgType>& message, const Buffer& buffer)
{
    std::cout << "   Type:               " << static_cast<int>(message.getType()) << std::endl;
    std::cout << "   Header size:        " << message.getHeaderSize()             << std::endl;
    std::cout << "   Body size:          " << message.getBodySize()               << std::endl;
    std::cout << "   Total message size: " << message.size()                      << std::endl;
    std::cout << "   Buffer size:        " << buffer.size()                       << std::endl;
}


Buffer 
serverOnReceiveCallback(const Buffer& buffer, const SocketAddress& client1Addr)
{
    Message<MessageType::TEXT> message;
    message << buffer;

    // ##### DEBUG PRINTS #####
    std::cout << "SERVER:" << std::endl;
    std::cout << "   Receive from:       " << (client1Addr.getIpAddress().has_value() ? client1Addr.getIpAddress().value() : "<unknown>") << ":" << client1Addr.getPort() << std::endl;
    printBufferHex(buffer);
    printSomeSizes(message, buffer);
    std::cout << "   MESSAGE:            " << message.getData().text << std::endl;
    // ##########

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return {};
}


int main() 
{
    pthread_setname_np(pthread_self(), "MainUdpTest");

    UdpServer server(8080);
    server.setName("UdpServ");
    server.setBufferSize(1024);
    server.setSocketBufferSize(1);
    server.setNoBlocking(true);
    server.setNoBlockingSleepTime(1);
    server.setClientHandlerSleepTime(1);
    server.setOnReceive(serverOnReceiveCallback);
    server.setMaxQueueMemoryUsage(24*10);
    server.start();

    std::vector<std::pair<UdpClient, Buffer>> clients;
    for (int i=1; i<=10; i++)
    {
        UdpClient client("127.0.0.1", (8080+i));
        client.setReceiverAddress("127.0.0.1", 8080);
        MessageData<MessageType::TEXT> data;
        data.text = std::string("Hello from client")+std::to_string(i)+std::string("!");
        Message<MessageType::TEXT> message(data);
        Buffer buffer;
        std::cout << "DEBUG: buffer size before serialize: " << buffer.size() << std::endl;
        message >> buffer;
        std::cout << "DEBUG: buffer size after serialize: " << buffer.size() << std::endl;
        clients.emplace_back(std::move(client), std::move(buffer));
        // ##### DEBUG PRINTS #####
        std::cout << "CLIENT:" << std::endl;
        std::cout << "   Send to:           " << (clients[i-1].first.getReceiverAddress().has_value() ? clients[i-1].first.getReceiverAddress().value() : "<unknown>") << ":" << clients[i-1].first.getReceiverPort() << std::endl;
        printBufferHex(clients[i-1].second);
        printSomeSizes(message, clients[i-1].second);
        std::cout << "   MESSAGE:            " << message.getData().text << std::endl;
        // ##########
    }

    for (auto& [client, buffer] : clients)
    {
        client.sendData(buffer);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for the server to recive and process message

    server.stop();
    return 0;
}
