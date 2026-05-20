#pragma once

#include <iostream>

#include "IObserver.h"
#include "SomeDataStruct.h"

class Observer
    : public IObserver<SomeDataStruct>
{
    public:
        Observer() = default;

        void update(const SomeDataStruct& data) override
        {
            std::cout << "Observed data: " << "text = " << data.text << ", "
                                            << "integer = " << data.integer << ", "
                                            << "number = " << data.number
                                            << std::endl;
        }
};