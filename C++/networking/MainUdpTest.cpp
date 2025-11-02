#include "UdpServer.h"
#include "UdpClient.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <arpa/inet.h>
#include "Message.h"
#include "MessageType.h"
#include "Serializer.h"

void 
printBufferHex(const Buffer& buffer)
{
    const char* data = buffer.getConstPtr();
    size_t size = buffer.size();
    std::cout << "   Buffer in HEX: ";
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
serverOnReceiveCallback(const Buffer& buffer, const SocketAddress& clientAddr)
{
    Message<MessageType::TEXT> message;
    message << buffer;

    // ##### DEBUG PRINTS #####
    std::cout << "SERVER:" << std::endl;
    std::cout << "   Receive from: " << (clientAddr.getIpAddress().has_value() ? clientAddr.getIpAddress().value() : "<unknown>") << ":" << clientAddr.getPort() << std::endl;
    printBufferHex(buffer);
    printSomeSizes(message, buffer);
    std::cout << "   MESSAGE: " << message.getData().text << std::endl;
    // ##########
    return {};
}


int main() 
{
    UdpServer server(8080);
    server.setNoBlocking(true);
    server.setOnReceive(serverOnReceiveCallback);
    server.start();

    // Create client and communicate
    UdpClient client("127.0.0.1", 8081);
    client.setReceiverAddress("127.0.0.1", 8080);

    MessageData<MessageType::TEXT> data;
    data.text = "Hei from client!";
    Message<MessageType::TEXT> message(data);
    Buffer buffer;
    buffer << message;
    client.sendData(buffer);
    // ##### DEBUG PRINTS #####
    std::cout << "CLIENT:" << std::endl;
    printSomeSizes(message, buffer);
    std::cout << "   MESSAGE: " << message.getData().text << std::endl;
    // ##########

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for the server to recive and process message

    server.stop();
    return 0;
}
