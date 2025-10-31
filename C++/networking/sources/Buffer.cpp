#include "Buffer.h"

Buffer::Buffer(size_t size)
    : size_(size)
    , bufferPtr_(new char[size])
{
    // Optional: initialize to zero
    std::memset(bufferPtr_, 0, size_);
}


Buffer::~Buffer()
{
    delete[] bufferPtr_;
}


Buffer::Buffer(Buffer &&other) noexcept
    : bufferPtr_(other.bufferPtr_), size_(other.size_)
{
    other.bufferPtr_ = nullptr;
    other.size_ = 0;
}


Buffer&
Buffer::operator=(Buffer &&other) noexcept
{
    if (this != &other) {
        delete[] bufferPtr_;
        bufferPtr_ = other.bufferPtr_;
        size_ = other.size_;
        other.bufferPtr_ = nullptr;
        other.size_ = 0;
    }
    return *this;
}


char&
Buffer::operator[](size_t i)
{ 
    if (i >= size_) throw std::out_of_range("Buffer index out of range");
    return bufferPtr_[i];
}
