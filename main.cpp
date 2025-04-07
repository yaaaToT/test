#include "include/Robot.hpp"
//#include "include/module/test/TestModule.hpp"
#include "include/module/BuffModule.hpp"
#include "include/pod/multiton/package/ReceiveDataPacket.hpp"
#include "include/pod/multiton/package/SendDataPacket.hpp"

int main() {
    Robot::getInstance()
        ->initCamera()
        ->initSerialPort()
        ->addReceivePacket(std::make_shared<ReceiveDataPacket>(0))
        ->addSendPacket(std::make_shared<SendDataPacket>(0))
        ->setupModule(std::make_shared<BuffModule>())
        ->startSystems();

    return 0;
}
