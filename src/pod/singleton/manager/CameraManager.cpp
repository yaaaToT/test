#include "../../../../include/pod/singleton/manager/CameraManager.hpp"

// CameraManager* CameraManager::instance = nullptr;
// std::mutex CameraManager::mtx;

// CameraManager & CameraManager::getInstance() {
//     if (instance == nullptr) {
//         std::lock_guard<std::mutex> lock(mtx); // 加锁以确保线程安全
//         if (instance == nullptr) {
//             instance = new CameraManager(); // 初始化时设置初始值
//         }
//     }
//     return *instance;
// }



// void CameraManager::addCamera(const std::shared_ptr<Camera>& cameraPtr) {
//     CameraManager::getInstance().mCameraPtrs.push_back(cameraPtr);
// }



// void CameraManager:: loadTasks(std::vector<std::thread>& tasks) {
//     for (const auto& cameraPtr : CameraManager::getInstance().mCameraPtrs) {

//         cameraPtr->startCamera();

//         tasks.emplace_back(&Camera::updateSrc, cameraPtr);
//     }
// }



// std::shared_ptr<Camera> CameraManager::getCameraById(const int& id) {
//     for (const auto& cameraPtr : mCameraPtrs) {
//         if (cameraPtr->getId() == id) {
//             return cameraPtr;
//         }
//     }
//     std::cout<<"does not exist camera: "<<id<<std::endl;
//     return nullptr;
// }





// int CameraManager::getCameraSize() {
//     return CameraManager::getInstance().mCameraPtrs.size();
// }


void CameraManager::addCamera(const std::shared_ptr<Camera>& cameraPtr) {
    this->mCameraPtrs.push_back(cameraPtr);
}


void CameraManager:: loadTasks(std::vector<std::thread>& tasks) {
    for (const std::shared_ptr<Camera>& cameraPtr : this->mCameraPtrs) {
        cameraPtr->startCamera();
        tasks.emplace_back([cameraPtr] { cameraPtr->updateSrc();});
    }
}



int CameraManager::getCameraSize() {
    return this->mCameraPtrs.size();
}
