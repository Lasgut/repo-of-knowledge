#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class MessageType : uint8_t
{
    TEXT        = 0x01,
    BINARY      = 0x02,
    COMMAND     = 0x03,
    HEARTBEAT   = 0x04
};


/**
 * @brief Custom message data for different message types.
 * 
 * This struct holds the data for a specific message type.
 * Each specialization corresponds to a different MessageType.
 *
 */
template<MessageType T>
struct MessageData;

template<>
struct MessageData<MessageType::TEXT> 
{
    static constexpr MessageType type = MessageType::TEXT;
    std::string text;
};

template<>
struct MessageData<MessageType::BINARY> 
{
    static constexpr MessageType type = MessageType::BINARY;
    std::vector<uint8_t> binaryVec;
};

template<>
struct MessageData<MessageType::COMMAND> 
{
    static constexpr MessageType type = MessageType::COMMAND;
    uint8_t commandId;
    std::string parameter;
};

template<>
struct MessageData<MessageType::HEARTBEAT> 
{
    static constexpr MessageType type = MessageType::HEARTBEAT;
    uint64_t timestamp;
};