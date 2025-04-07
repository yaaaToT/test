#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <mutex>
//#include <thread>
//#include "../multiton/package/ReceiveDataPacket.hpp"
//#include "../multiton/package/SendDataPacket.hpp"
#include "../multiton/package/DataPacket.hpp"


class SerialPort {
public:
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(const SerialPort&) = delete;
    SerialPort() = default;
    void init();
    bool openDev();
    bool setSpeed();
    bool setParity(int databits, int stopbits, int parity);
    bool updateDevList();
    static std::shared_ptr<SerialPort> getInstance();
    static int mFd;
private:
    static std::vector<std::string> mDeviceNames;
    static termios options;
    static std::shared_ptr<SerialPort> instance;
    static std::mutex mtx;
    static int mBaudSpeed;
};





// class SerialPort {
// public:
//     SerialPort();
//     static SerialPort& getInstance();


//     void setSpeed(int fd, speed_t speed);
//     int setParity(int fd, int databits, int stopbits, int parity);
//     void loadTasks(std::vector<std::thread>& tasks);
//     void startSendDataPacket();
//     void startReceiveDataPacket();


//     static int mFd;
//     static int mBaudSpeed;
//     std::vector<std::pair<int, ReceiveDataPacket>> mReceiveDataPackets;
//     std::vector<std::pair<int, SendDataPacket>> mSendDataPackets;
// private:
//     SerialPort& operator=(const SerialPort&) = delete;
//     SerialPort(const SerialPort&) = delete;
//     static struct termios options;
//     static SerialPort* instance;
//     static std::mutex mtx;
// };


#endif //SERIALPORT_HPP
