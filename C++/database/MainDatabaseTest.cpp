#include <iostream>
#include <thread>
#include <chrono>

#include "Database.h"
#include "Observer.h"

int main() 
{
    std::cout << "hello" << std::endl;

    // push data to database
    SomeDataStruct data;
    data.text    = "hello";
    data.integer = 1;
    data.number  = 1.1;
    Database::instance().push<SomeDataStruct>(data);

    // pull data from database
    auto pulledDataOpt = Database::instance().pull<SomeDataStruct>();
    if (pulledDataOpt.has_value())
    {
        auto pulledData = pulledDataOpt.value();
        std::cout << "Pulled data: " << "text = " << pulledData.text << ", "
                                    << "integer = " << pulledData.integer << ", "
                                    << "number = " << pulledData.number
                                    << std::endl;
    }

    // thread that continuously pushes updated data every second
    std::thread constantPush([data]() mutable {
        while (true) 
        {
            data.integer += 1;
            Database::instance().push(data);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // thread that continuously pulls updated data every second
    std::thread constantPull([]() {
        while (true) 
        {
            auto pulledDataOpt = Database::instance().pull<SomeDataStruct>();
            if (pulledDataOpt.has_value())
            {
                auto pulledData = pulledDataOpt.value();
                std::cout << "Pulled data: " << "text = " << pulledData.text << ", "
                                            << "integer = " << pulledData.integer << ", "
                                            << "number = " << pulledData.number
                                            << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    Observer observer;
    Database::instance().addObserver<SomeDataStruct>(&observer);

    while (true)
    {

    }

}