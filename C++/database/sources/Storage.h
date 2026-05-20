#pragma once

#include <algorithm>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "SomeDataStruct.h"
#include "IObserver.h"



template <typename DataType>
class Storage
{
    public:
        Storage() = default;

        void                    push(const DataType& data);
        std::optional<DataType> pull();
        void                    reset();

        void addObserver(IObserver<DataType>* observer);
        void removeObserver(IObserver<DataType>* observer);

        void notifyObservers();

    private:
        std::vector<IObserver<DataType>*> observers_;
        std::optional<DataType>           data_;
        std::mutex                        mutex_;
};


template <typename DataType>
inline void 
Storage<DataType>::push(const DataType& data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
    notifyObservers();
}


template <typename DataType>
inline std::optional<DataType> 
Storage<DataType>::pull()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
}

template <typename DataType>
inline void 
Storage<DataType>::reset()
{
    data_.reset();
}

template <typename DataType>
inline void 
Storage<DataType>::addObserver(IObserver<DataType> *observer)
{
    observers_.push_back(observer);
}


template <typename DataType>
inline void 
Storage<DataType>::removeObserver(IObserver<DataType> *observer)
{
    observers_.erase(std::remove(observers_.begin(), observers_.end(), observer), observers_.end());
}


template <typename DataType>
inline void 
Storage<DataType>::notifyObservers()
{
    if (not data_) 
    {
        return;
    }

    for (auto* observer : observers_) 
    {
        observer->update(*data_);
    }
}
