#pragma once

template <typename DataType>
class IObserver
{
public:
    virtual ~IObserver() = default;
    virtual void update(const DataType& data) = 0;
};