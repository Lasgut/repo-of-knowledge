#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class MessageType : uint8_t
{
    TEXT        = 0x01,
    MIX       = 0x02,
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

    template<typename F>
    void visitFields(F&& f) const {
        f(text);
    }

    template<typename F>
    void visitFields(F&& f) {
        f(text);
    }
};


template<>
struct MessageData<MessageType::MIX> 
{
    static constexpr MessageType type = MessageType::MIX;
    std::string text;
    uint16_t    uint16;
    uint32_t    uint32;
    int64_t     int64;

    template<typename F>
    void visitFields(F&& f) const {
        f(text);
        f(uint16);
        f(uint32);
        f(int64);
    }

    template<typename F>
    void visitFields(F&& f) {
        f(text);
        f(uint16);
        f(uint32);
        f(int64);
    }
};