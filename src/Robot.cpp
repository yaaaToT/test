#include <filesystem>
#include "../include/Robot.hpp"
#include "../include/pod/singleton/SerialPort.hpp"
#include "../include/pod/multiton/camera/DahuaCamera.hpp"
#include "../include/pod/multiton/camera/MindCamera.hpp"
#include "../include/pod/multiton/camera/VirtualCamera.hpp"
#include "../include/pod/multiton/camera/VideoCamera.hpp"
//#include "../config/config.yml"

// 定义静态成员变量
std::shared_ptr<Robot> Robot::robot = nullptr;

// 启动系统的方法
void Robot::startSystems() {
    // 加载任务到摄像头管理器
    mCameraManager.loadTasks(this->tasks);
    // 加载任务到模块管理器
    mModuleManager.loadTasks(this->tasks);
    // 加载接收任务到接收数据包管理器
    mReceivePacketManager.loadReceiveTasks(this->tasks);
    // 加载发送任务到发送数据包管理器
    mSendPacketManager.loadSendTasks(this->tasks);
    // 输出任务数量
    std::cout<<"tasks number: "<<this->tasks.size()<<std::endl;
    // 等待所有任务完成
    for(auto& task : this->tasks) {
        task.join();
    }

}

Robot::Robot() {
    //输出相对路径
    std::cout<<std::filesystem::current_path()<<std::endl;
}

// 初始化串口的方法
std::shared_ptr<Robot> Robot::initSerialPort() {
    SerialPort::getInstance()->init();
    return robot;
}



std::shared_ptr<Robot> Robot::initCamera() {

    // 读取配置文件
    //cv::FileStorage fileReader("../../config/config.yml",cv::FileStorage::READ);
    cv::FileStorage fileReader("../config/config.yml",cv::FileStorage::READ);

    //初始化相机
    std::vector<int> cameraIds;
    fileReader["Settings"]["CameraIds"] >> cameraIds;
    fileReader.release();
    // 遍历摄像头ID并加载配置
    for (const int& cameraId : cameraIds) {
        //std::string fileName = "../config/camera/" + std::to_string(cameraId) +".yml";
        std::string fileName = "../config/camera/" + std::to_string(cameraId) +".yml";

        cv::FileStorage cameraFileReader(fileName,cv::FileStorage::READ);
        CameraType cameraType;
        cameraFileReader["CameraType"] >> cameraType;



        switch (cameraType) {
            case DAHUA:
//                mCameraManager.addCamera(std::make_shared<DahuaCamera>(cameraFileReader,cameraId));
                break;
            case MIND:
                mCameraManager.addCamera(std::make_shared<MindCamera>(cameraFileReader,cameraId));
                break;
            case DAHENG:
                // CameraManager::getInstance().addCamera(std::make_shared<DahengCamera>(cameraFileReader,cameraId));
                break;
            case VIRTUAL:
                mCameraManager.addCamera(std::make_shared<VirtualCamera>(cameraFileReader,cameraId));
                break;
            case VIDEO:
                mCameraManager.addCamera(std::make_shared<VideoCamera>(cameraFileReader,cameraId));
                break;
            default:
                break;
        }
        cameraFileReader.release();
    }

    if(!cameraIds.empty()) {
        std::cout << "read " << cameraIds.size() << " camera configs : ";
        for (const int& cameraId : cameraIds) std::cout << cameraId << " ";
        std::cout << std::endl;
        std::cout << "loaded " << mCameraManager.getCameraSize() <<" cameras"<<std::endl;
    }else std::cout << "read " << cameraIds.size() << " camera configs"<<std::endl;

    return getInstance();
}

// 返回当前的Robot实例
std::shared_ptr<Robot> Robot::setupModule(const std::shared_ptr<TorosamyModule>& modulePtr) {
    this->mModuleManager.addModule(modulePtr);
    return robot;
}
// 添加模块到模块管理器
std::shared_ptr<Robot> Robot::addReceivePacket(const std::shared_ptr<DataPacket>& packetPtr) {
    this->mReceivePacketManager.addPacket(packetPtr);
    // 返回当前的Robot实例
    return robot;
}

// 添加接收数据包的方法
std::shared_ptr<Robot> Robot::addSendPacket(const std::shared_ptr<DataPacket>& packetPtr) {
    // 添加数据包到接收数据包管理器
    this->mSendPacketManager.addPacket(packetPtr);
    // 返回当前的Robot实例
    return robot;
}


// 获取发送数据包管理器的方法
PacketManager& Robot::getSendPacketManager() {
    // 返回发送数据包管理器的引用
    return mSendPacketManager;
}

// 获取接收数据包管理器的方法
PacketManager& Robot::getReceivePacketManager() {
    // 返回接收数据包管理器的引用
    return mReceivePacketManager;
}

// 获取模块管理器的方法
ModuleManager& Robot::getModuleManager() {
    // 返回模块管理器的引用
    return mModuleManager;
}

// 获取摄像头管理器的方法
CameraManager& Robot::getCameraManager() {
    // 返回摄像头管理器的引用
    return mCameraManager;
}

// 获取单例实例的方法
std::shared_ptr<Robot> Robot::getInstance() {
    // 如果实例不存在，则创建一个新实例
    if (!robot) {
        robot = std::make_shared<Robot>();
    }
    // 返回实例
    return robot;
}


// void Robot::initGun(const cv::FileStorage& fileReader) {
//     //初始化收发
//     std::vector<int> gunIds;
//     fileReader["Settings"]["GunIds"] >> gunIds;



//     for (const int& id : gunIds) {
//         std::string gunIdString = std::to_string(id);
//         SendDataPacket sendDataPacket;
//         fileReader["Settings"][gunIdString]["SendPacket"]["Head"] >> sendDataPacket.mHead;
//         fileReader["Settings"][gunIdString]["SendPacket"]["Tail"] >> sendDataPacket.mTail;
//         ReceiveDataPacket receiveDataPacket;
//         fileReader["Settings"][gunIdString]["ReceivePacket"]["Head"] >> receiveDataPacket.mHead;
//         fileReader["Settings"][gunIdString]["ReceivePacket"]["Tail"] >> receiveDataPacket.mTail;

//         SerialPort::getInstance().mSendDataPackets.push_back({id, sendDataPacket});
//         SerialPort::getInstance().mReceiveDataPackets.push_back({id, receiveDataPacket});
//     }

//     if(!gunIds.empty()) {
//         std::cout << "loaded " << gunIds.size() <<" gun configs : ";
//         for (const int& id : gunIds) std::cout << id << " ";
//         std::cout<<std::endl;

//         std::cout<<"load " << SerialPort::getInstance().mReceiveDataPackets.size() << " receive data packets : ";
//         for (const std::pair<int, ReceiveDataPacket>& pair : SerialPort::getInstance().mReceiveDataPackets) {
//             std::cout << pair.first << " ";
//         }
//         std::cout<<std::endl;


//         std::cout << "load " << SerialPort::getInstance().mSendDataPackets.size() << " send data packets : ";
//         for (const std::pair<int, SendDataPacket>& pair : SerialPort::getInstance().mSendDataPackets) {
//             std::cout << pair.first << " ";
//         }
//         std::cout<<std::endl;
//     }else std::cout << "loaded " << gunIds.size() <<" gun configs" << std::endl;

// }



// Robot& Robot::setupModule(const std::shared_ptr<TorosamyModule>& modulePtr) {
//     this->mModuleManager.addModule(modulePtr);
//     return *this;
// }



