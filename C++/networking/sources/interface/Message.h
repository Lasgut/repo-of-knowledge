#pragma once

#include "MessageType.h"
#include "Buffer.h"
#include "Serializer.h"

template<MessageType MsgType>
class Message
    : Serializer
{
public:
    using DataType = MessageData<MsgType>;

    Message();
    Message(const DataType& data);

    void setData(const DataType& data);

    const DataType&    getData()       const noexcept { return body_.data; }
    const MessageType& getType()       const noexcept { return header_.type; }
    const size_t       getHeaderSize() const noexcept { return sizeof(header_.type) + sizeof(header_.bodySize); }
    const size_t       getBodySize()   const noexcept { return header_.bodySize; }
    const char*        getHeaderPtr()  const noexcept { return reinterpret_cast<const char*>(&header_); }

    size_t size() const noexcept;


    const Message<MsgType>& operator>>(Buffer& buffer)
    {
        if (buffer.size() != size())
        {
            buffer.resize(size());
        }
        serialize(getHeaderPtr(), getHeaderSize(), getData(), buffer);
        return *this;
    }

    Message<MsgType>& operator<<(const Buffer& buffer)
    {
        //the first byte in the buffer is the messagetype
        MessageType typeInBuffer = *reinterpret_cast<const MessageType*>(buffer.getConstPtr());
        if (typeInBuffer != MsgType)
        {
            throw std::runtime_error("Message type mismatch during deserialization.");
        }
        header_.bodySize = static_cast<uint32_t>(buffer.size() - getHeaderSize());
        deserialize(buffer, body_.data, getHeaderSize());
        return *this;
    }

private:
    struct Header
    {
        MessageType type{MsgType};
        uint32_t    bodySize{0}; // size of the body
    };

    struct Body
    {
        DataType data{};
    };

    Header header_{};
    Body   body_{};
};


template <MessageType MsgType>
inline Message<MsgType>::Message()
{
    header_.type     = MsgType;
    header_.bodySize = 0;
}


template <MessageType MsgType>
inline Message<MsgType>::Message(const DataType &data)
{
    body_.data       = data;
    header_.type     = MsgType;
    header_.bodySize = getSerializedSize(body_.data);
}


template <MessageType MsgType>
inline void 
Message<MsgType>::setData(const DataType &data)
{
    body_.data       = data;
    header_.bodySize = getSerializedSize(body_.data);
}


template <MessageType MsgType>
inline size_t 
Message<MsgType>::size() const noexcept
{
    return getHeaderSize() + getBodySize();
}
