#pragma once

#include "MessageType.h"
#include "Buffer.h"

#include <cstring>
#include <vector>
#include <string>


class Serializer
{
protected:
    template<MessageType MsgType>
    static void 
    serialize(const char* headerPtr, size_t headerSize, const MessageData<MsgType>& data, Buffer& buffer)
    {
        // --- Serialize header ---
        size_t offset = 0;
        //const size_t headerSize = sizeof(MessageType) + sizeof(uint32_t);
        std::memcpy(buffer.getPtr() + offset, headerPtr, headerSize);
        offset += headerSize;

        // --- Serialize body based on message type ---
        serializeData(data, buffer, offset);
    }

    template<MessageType MsgType>
    static void 
    deserialize(const Buffer& buffer, MessageData<MsgType>& data, size_t headerSize)
    {
        deserializeData(data, buffer, headerSize);
    }

    static size_t 
    getSerializedSize(const MessageData<MessageType::TEXT>& data)
    {
        return data.text.size();
    }

private:
    // --- TEXT ---
    static void 
    serializeData(const MessageData<MessageType::TEXT>& data, Buffer& buffer, size_t offset)
    {
        auto len = getSerializedSize(data);
        std::memcpy(buffer.getPtr() + offset, data.text.data(), len);
    }

    static void 
    deserializeData(MessageData<MessageType::TEXT>& data, const Buffer& buffer, size_t offset)
    {
        size_t len = buffer.size() - offset;
        data.text.assign(buffer.getConstPtr() + offset, len);
    }
};
