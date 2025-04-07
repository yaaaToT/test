#include "../../../../include/pod/singleton/manager/ModuleManager.hpp"



void ModuleManager::addModule(const std::shared_ptr<TorosamyModule>& modulePtr) {
    mModulePtrs.push_back(modulePtr);
}


void ModuleManager::loadTasks(std::vector<std::thread>& tasks) {
    for (const std::shared_ptr<TorosamyModule>& modulePtr : mModulePtrs) {
        tasks.emplace_back([modulePtr] {modulePtr->run();});
    }
}

int ModuleManager::getSize() const{
    return mModulePtrs.size();
}