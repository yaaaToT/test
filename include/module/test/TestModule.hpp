//
// Created by torosamy on 24-11-18.
//

#ifndef TESTMODULE_HPP
#define TESTMODULE_HPP
#include "../TorosamyModule.hpp"

class TestModule : public TorosamyModule {
public:
    TestModule();
    void run() override;
};


#endif //TESTMODULE_HPP
