#include "../../include/module/TorosamyModule.hpp"
//#include "../../config/config.yml"
//#include "../../config/module/test.yml"
std::string TorosamyModule::getConfigLocation(const std::string& moduleName) {
     return "../config/module/" +moduleName + ".yml";
}

