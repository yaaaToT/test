#ifndef MODULEMANAGER_HPP
#define MODULEMANAGER_HPP
#include <memory>
#include <thread>

#include "../../../module/TorosamyModule.hpp"

class ModuleManager {
public:
    ModuleManager() = default;
    void addModule(const std::shared_ptr<TorosamyModule>& modulePtr);
    void loadTasks(std::vector<std::thread>& tasks);
    int getSize() const;
private:
    std::vector<std::shared_ptr<TorosamyModule>> mModulePtrs;
};



#endif //MODULEMANAGER_HPP
