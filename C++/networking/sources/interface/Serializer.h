#pragma once

#include "MessageType.h"
#include "Buffer.h"

#include <cstring>
#include <vector>
#include <string>


class Serializer
{
protected:
    template<typename MsgType>
    void 
    serialize(const char* headerPtr, size_t headerSize,
                          const MsgType& data, Buffer& buffer)
    {
        int offset = 0;
        std::memcpy(buffer.getPtr(), headerPtr, headerSize);
        offset += headerSize;

        data.visitFields([&](auto& field) {
            serializeValue(field, buffer, offset);
        });
    }

    template<typename MsgType>
    void 
    deserialize(const Buffer& buffer, MsgType& data, size_t headerSize)
    {
        int offset = headerSize;
        data.visitFields([&](auto& field) {
            deserializeValue(field, buffer, offset);
        });
    }

    template<typename MsgType>
    size_t 
    getSerializedSize(const MsgType& data)
    {
        int size = 0;
        data.visitFields([&](auto& field) {
            getValueSize(field, size);
        });
        return size;
    }

private:
    // --- std::string ---
    void 
    serializeValue(const std::string& value, Buffer& buffer, int& offset)
    {
        auto len = value.size();
        std::memcpy(buffer.getPtr() + offset, value.data(), len);
        offset += len;
    }

    void 
    deserializeValue(std::string& value, const Buffer& buffer, int& offset)
    {
        size_t len = buffer.size() - offset;
        value.assign(buffer.getConstPtr() + offset, len);
        offset += len;
    }

    void 
    getValueSize(const std::string& value, int& size)
    {
        size += value.size();
    }
    // --------------------

    // --- General ---
    template<typename T>
    void 
    serializeValue(const T& value, Buffer& buffer, int& offset)
    {
        std::memcpy(buffer.getPtr() + offset, &value, sizeof(value));
        offset += sizeof(value);
    }   

    template<typename T>
    void 
    deserializeValue(T& value, const Buffer& buffer, int& offset)
    {
        std::memcpy(&value, buffer.getConstPtr() + offset, sizeof(value));
        offset += sizeof(value);
    }

    template<typename T>
    void 
    getValueSize(const T& value, int& size)
    {
        size += sizeof(value);
    }
    // --------------------
};
