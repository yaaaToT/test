#ifndef TOROSAMYMODULE_HPP
#define TOROSAMYMODULE_HPP
#include <iostream>
#include <vector>


class TorosamyModule {
public:
    virtual void run() = 0;
    virtual ~TorosamyModule() = default;
    static std::string getConfigLocation(const std::string& moduleName);

protected:
    static std::vector<std::string> moduleNames;
};


#endif //TOROSAMYMODULE_HPP
