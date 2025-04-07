#ifndef CAMEARMANAGER_HPP
#define CAMEARMANAGER_HPP
#include <thread>
#include <vector>
#include "../../multiton/camera/Camera.hpp"

class CameraManager {
public:
    CameraManager() = default;
    void addCamera(const std::shared_ptr<Camera>& cameraPtr);
    int getCameraSize();
    void loadTasks(std::vector<std::thread>& tasks);

    template <typename T>
    std::shared_ptr<T> getCameraById(const int& id);
private:
    std::vector<std::shared_ptr<Camera>> mCameraPtrs;
};

#include "CameraManager.tpp"
#endif //CAMEARMANAGER_HPP

// class CameraManager {
// public:
//     //static CameraManager& getInstance();
//     //void addCamera(const std::shared_ptr<Camera>& cameraPtr);
//     std::shared_ptr<Camera> getCameraById(const int& id);
//     int getCameraSize();
//     void loadTasks(std::vector<std::thread>& tasks);
// private:
//     CameraManager() = default;
//     CameraManager(const CameraManager&) = delete;
//     CameraManager& operator=(const CameraManager&) = delete;

//     static CameraManager* instance;
//     static std::mutex mtx;
//     std::vector<std::shared_ptr<Camera>> mCameraPtrs;
// };



