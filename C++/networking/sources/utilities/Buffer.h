#pragma once
#include <cstddef>
#include <cstring>
#include <stdexcept>

class Buffer 
{
    public:
        Buffer() = default;
        explicit Buffer(size_t size);
        ~Buffer();
        Buffer(Buffer&& other) noexcept;             // move constructor
        Buffer(const Buffer&)              = delete; // no copy
        Buffer& operator=(const Buffer&)   = delete; // no copy assign
        Buffer&& operator=(const Buffer&&) = delete; // no move assign

        // Access raw pointer
        char*       getPtr()      noexcept       { return bufferPtr_; }
        const char* getConstPtr() const noexcept { return bufferPtr_; }

        size_t size() const noexcept { return size_; }
        void   resize(size_t newSize);
        void   clear();

        // Optional: operators for convenience
        Buffer& operator=(Buffer&& other) noexcept;
        char&   operator[](size_t i);

    private:
        char*  bufferPtr_{nullptr};
        size_t size_{0};
};
