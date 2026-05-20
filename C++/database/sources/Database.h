#pragma once

#include <unordered_map>
#include <typeindex>
#include <memory>
#include <any>

#include "Storage.h"
#include "SomeDataStruct.h"

class Database
{
    public:
        static Database& instance()
        {
            static Database instance;
            return instance;
        }

        // non-copyable, non-movable
        Database(const Database&)            = delete;
        Database& operator=(const Database&) = delete;
        Database(Database&&)                 = delete;
        Database& operator=(Database&&)      = delete;

        template<typename T>
        void push(const T& data);

        template<typename T>
        std::optional<T> pull();

        template<typename T>
        void addObserver(IObserver<T>* observer);

    private:
        Database()  = default;
        ~Database() = default;

        std::unordered_map<std::type_index, std::shared_ptr<void>> storages_;

        std::mutex mutex_;

        template<typename T>
        Storage<T>& getStorage();
};


template <typename T>
inline void 
Database::push(const T& data)
{
    getStorage<T>().push(data);
}


template <typename T>
inline std::optional<T> 
Database::pull()
{
    return getStorage<T>().pull();
}

template <typename T>
inline void 
Database::addObserver(IObserver<T> *observer)
{
    getStorage<T>().addObserver(observer);
}


template <typename T>
inline Storage<T>&
Database::getStorage()
{
    std::lock_guard lock(mutex_);

    auto type = std::type_index(typeid(T));
    auto it   = storages_.find(type);

    if (it == storages_.end())
    {
        auto storage    = std::make_shared<Storage<T>>();
        storages_[type] = storage;
        return *storage;
    }

    return *static_cast<Storage<T>*>(it->second.get());
}
