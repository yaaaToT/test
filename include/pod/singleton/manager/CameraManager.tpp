#ifndef CAMEARMANAGER_TPP
#define CAMEARMANAGER_TPP

#include "../../../../include/pod/singleton/manager/CameraManager.hpp"
//#include "CameraManager.hpp"
// 模板函数定义
template <typename T>
std::shared_ptr<T> CameraManager::getCameraById(const int& id) {

    for (const auto& ptr : mCameraPtrs) {
        if (ptr->getId() == id) {
            return std::dynamic_pointer_cast<T>(ptr);
        }
    }

    try {
        throw std::out_of_range("camera not found with id: " + std::to_string(id));
    } catch (const std::out_of_range& e) {
        std::cerr << e.what() << std::endl;
    }

}

#endif  // CAMEARMANAGER_TPP
