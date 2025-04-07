#ifndef ROBOT_HPP
#define ROBOT_HPP


#include "pod/singleton/manager/ModuleManager.hpp"
#include "pod/singleton/manager/CameraManager.hpp"
#include "pod/multiton/package/DataPacket.hpp"
#include "pod/singleton/manager/PacketManager.hpp"

class Robot {
public:
    Robot();

    std::shared_ptr<Robot> setupModule(const std::shared_ptr<TorosamyModule>& modulePtr);

    // 添加接收数据包
    std::shared_ptr<Robot> addSendPacket(const std::shared_ptr<DataPacket>& packetPtr);
    // 添加发送数据包
    std::shared_ptr<Robot> addReceivePacket(const std::shared_ptr<DataPacket>& packetPtr);
    // 初始化串口
    std::shared_ptr<Robot> initSerialPort();
    // 初始化摄像头
    std::shared_ptr<Robot> initCamera();
    // 启动系统
    void startSystems();
    // 获取发送数据包管理器
    PacketManager& getSendPacketManager();
    // 获取接收数据包管理器
    PacketManager& getReceivePacketManager();
    // 获取模块管理器
    ModuleManager& getModuleManager();
    // 获取摄像头管理器
    CameraManager& getCameraManager();

   // 获取单例实例
    static std::shared_ptr<Robot> getInstance();
private:
    static std::shared_ptr<Robot> robot;
    PacketManager mSendPacketManager;
    PacketManager mReceivePacketManager;
    CameraManager mCameraManager;
    ModuleManager mModuleManager;
    std::vector<std::thread> tasks;
};


#endif //ROBOT_HPP
