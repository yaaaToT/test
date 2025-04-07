#include "../../../include/pod/singleton/SerialPort.hpp"
//#include "configfilereader.h"
#include <fcntl.h>
#include <dirent.h>
#include<opencv2/opencv.hpp>
//#include "../../../config/config.yml"
// // 静态成员初始化
// SerialPort* SerialPort::instance = nullptr;
// std::mutex SerialPort::mtx;
// struct termios SerialPort::options;
// int SerialPort::mBaudSpeed;
// int SerialPort::mFd;


// void SerialPort::loadTasks(std::vector<std::thread>& tasks) {
//     tasks.emplace_back(&SerialPort::startSendDataPacket, this);
//     tasks.emplace_back(&SerialPort::startReceiveDataPacket, this);
// }




// void SerialPort::startSendDataPacket() {
//     while(true) {
//         SendDataPacket::sendDataPacket(mFd, mSendDataPackets);
//     }


// }

// void SerialPort::startReceiveDataPacket() {
//     while (true) {
//         ReceiveDataPacket::receiveDataPacket(mFd,mReceiveDataPackets);
//     }

// }



// SerialPort& SerialPort::getInstance() {
//     if (instance == nullptr) {
//         std::lock_guard<std::mutex> lock(mtx);
//         if (instance == nullptr) {
//             instance = new SerialPort();
//         }
//     }
//     return *instance;
// }


// SerialPort::SerialPort() {
//     int fdcom = 0;
//     if (open("/dev/ttyACM0", O_RDWR | O_NONBLOCK) == -1) {
//         std::cout << "/dev/ttyACM1" << std::endl;
//         fdcom = open("/dev/ttyACM1", O_RDWR | O_NONBLOCK); // O_NOCTTY | O_NDELAY
//     }
//     else {
//         std::cout << "/dev/ttyACM0" << std::endl;
//         fdcom = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK); // O_NOCTTY | O_NDELAY
//     }
//     if (-1 == fdcom)  {
//         perror("can''t open serial port\n");
//         return;
//     }


//     perror("open serial port success\n");
//     setSpeed(fdcom, mBaudSpeed);
//     if (setParity(fdcom, 8, 1, 'N') == -1) {
//         printf("set parity error\n");
//         exit(0);
//     }



// }

// void SerialPort::setSpeed(int fd, speed_t speed) {
//     struct termios Opt;
//     tcgetattr(fd, &Opt);
//     for (int i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
//         if (speed == name_arr[i]) {
//             tcflush(fd, TCIOFLUSH);
//             cfsetispeed(&Opt, speed_arr[i]);
//             cfsetospeed(&Opt, speed_arr[i]);
//             if (tcsetattr(fd, TCSANOW, &Opt) != 0) {
//                 perror("tcsetattr fd1");
//                 return;
//             }
//             tcflush(fd, TCIOFLUSH);
//         }
//     }
// }


// int SerialPort::setParity(int fd, int databits, int stopbits, int parity) {
//     if (tcgetattr(fd, &options) != 0){
//         perror("setup serial 1");
//         return -1;
//     }

//     options.c_cflag &= ~static_cast<unsigned int>(CSIZE);
//     switch (databits) /*设置数据位数*/
//     {
//     case 7:
//         options.c_cflag |= CS7;
//         break;
//     case 8:
//         options.c_cflag |= CS8;
//         break;
//     default:
//         fprintf(stderr, "unsupported data sizen");
//         return -1;
//     }

//     switch (parity)
//     {
//     case 'N':
//     {
//         options.c_cflag &= ~static_cast<unsigned int>(PARENB); /* Clear parity enable */
//         options.c_iflag &= ~static_cast<unsigned int>(INPCK);  /* Enable parity checking */
//     }
//     break;

//     case 'O':
//     {
//         options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
//         options.c_iflag |= INPCK;             /* Disnable parity checking */
//     }
//     break;

//     case 'E':
//     {
//         options.c_cflag |= PARENB;                             /* Enable parity */
//         options.c_cflag &= ~static_cast<unsigned int>(PARODD); /* 转换为偶效验*/
//         options.c_iflag |= INPCK;                              /* Disnable parity checking */
//     }
//     break;

//     case 'S': /*as no parity*/
//     {
//         options.c_cflag &= ~static_cast<unsigned int>(PARENB);
//         options.c_cflag &= ~static_cast<unsigned int>(CSTOPB);
//     }
//     break;

//     default:
//     {
//         fprintf(stderr, "unsupported parityn");
//         return -1;
//     }
//     }
//     /* 设置停止位*/
//     switch (stopbits)
//     {
//     case 1:
//         options.c_cflag &= ~static_cast<unsigned int>(CSTOPB);
//         break;
//     case 2:
//         options.c_cflag |= CSTOPB;
//         break;
//     default:
//         fprintf(stderr, "unsupported stop bitsn");
//         return -1;
//     }
//     /* Set input parity option */
//     if (parity != 'n') options.c_iflag |= INPCK;

//     options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
//     options.c_cc[VMIN]  = 0;   /* Update the options and do it NOW */

//     options.c_cflag &= ~static_cast<unsigned int>(HUPCL);
//     options.c_iflag &= ~static_cast<unsigned int>(INPCK);
//     options.c_iflag |= static_cast<unsigned int>(IGNBRK);
//     options.c_iflag &= ~static_cast<unsigned int>(ICRNL);
//     options.c_iflag &= ~static_cast<unsigned int>(IXON);
//     options.c_lflag &= static_cast<unsigned int>(IEXTEN);
//     options.c_lflag &= ~static_cast<unsigned int>(ECHOK);
//     options.c_lflag &= ~static_cast<unsigned int>(ECHOCTL);
//     options.c_lflag &= ~static_cast<unsigned int>(ECHOKE);
//     options.c_lflag &= ~static_cast<unsigned int>(ONLCR);
//     options.c_oflag = ~static_cast<unsigned int>(ICANON);

//     tcflush(fd, TCIFLUSH);
//     if (tcsetattr(fd, TCSANOW, &options) != 0) {
//         perror("setup serial 3");
//         return -1;
//     }
//     SerialPort::mFd = fd;
//     return 0;
// }


// 静态成员初始化
std::shared_ptr<SerialPort> SerialPort::instance = nullptr;
std::mutex SerialPort::mtx;
termios SerialPort::options;
int SerialPort::mBaudSpeed;
int SerialPort::mFd;
std::vector<std::string> SerialPort::mDeviceNames;

std::shared_ptr<SerialPort> SerialPort::getInstance() {
    if (instance == nullptr) {
        std::lock_guard lock(mtx);
        instance = std::make_shared<SerialPort>();
    }
    return instance;
}


void SerialPort::init() {
    //cv::FileStorage fileReader("../../../config/config.yml",cv::FileStorage::READ);
    cv::FileStorage fileReader("../config/config.yml",cv::FileStorage::READ);

    fileReader["Settings"]["BaudSpeed"] >> mBaudSpeed;
    fileReader.release();

    std::cout<<"serial port baud speed : "<<mBaudSpeed<<std::endl;

    if(!updateDevList()) return;

    while(true) {
        if (!openDev()) {
            perror("can't open serial port\n");
            continue;
        }
        if (!setSpeed()) {
            perror("set speed failed\n");
            continue;
        }
        if (!setParity(8, 1, 'N')) {
            perror("set parity error\n");
            continue;
        }
        break;
    }
}
bool SerialPort::updateDevList() {
    std::string directory = "/dev";
    std::string keyword = "ttyACM";
    DIR* dir = opendir(directory.c_str());
    if (dir == nullptr) {
        std::cerr << "could not open directory: " << directory << std::endl;
        return false;
    }

    std::vector<std::string> files;
    dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename(entry->d_name);

        if (filename == "." || filename == "..") continue;
        if (filename.find(keyword) != std::string::npos) files.push_back(filename);
    }
    closedir(dir);

    if(files.empty()) {
        std::cout << "no dev found" << std::endl;
        mDeviceNames.clear();
        return false;
    }

    std::cout<<"found dev:";
    for (const auto& file : files) std::cout << " " << file;
    std::cout << std::endl;
    mDeviceNames = files;
    return true;
}

bool SerialPort::openDev() {
    if(mDeviceNames.empty()) return false;

    for (const auto& deviceName : mDeviceNames) {
        std::string devicePath = "/dev/" + deviceName;
        mFd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);

        std::cout << "open dev:" << devicePath << std::endl;
        if(mFd != -1) return true;
    }
    return false;
}


bool SerialPort::setSpeed() {
    struct termios Opt;
    tcgetattr(mFd, &Opt);
    for (int i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        if (mBaudSpeed == name_arr[i]) {
            tcflush(mFd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            if (tcsetattr(mFd, TCSANOW, &Opt) != 0) {
                perror("tcsetattr fd1");
                return false;
            }
            tcflush(mFd, TCIOFLUSH);
        }
    }
    return true;
}


bool SerialPort::setParity(int databits, int stopbits, int parity) {
    if (tcgetattr(mFd, &options) != 0){
        perror("setup serial 1");
        return false;
    }

    options.c_cflag &= ~static_cast<unsigned int>(CSIZE);
    switch (databits) /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "unsupported data sizen");
        return false;
    }

    switch (parity)
    {
    case 'N':
    {
        options.c_cflag &= ~static_cast<unsigned int>(PARENB); /* Clear parity enable */
        options.c_iflag &= ~static_cast<unsigned int>(INPCK);  /* Enable parity checking */
    }
    break;

    case 'O':
    {
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
        options.c_iflag |= INPCK;             /* Disnable parity checking */
    }
    break;

    case 'E':
    {
        options.c_cflag |= PARENB;                             /* Enable parity */
        options.c_cflag &= ~static_cast<unsigned int>(PARODD); /* 转换为偶效验*/
        options.c_iflag |= INPCK;                              /* Disnable parity checking */
    }
    break;

    case 'S': /*as no parity*/
    {
        options.c_cflag &= ~static_cast<unsigned int>(PARENB);
        options.c_cflag &= ~static_cast<unsigned int>(CSTOPB);
    }
    break;

    default:
    {
        fprintf(stderr, "unsupported parityn");
        return false;
    }
    }
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~static_cast<unsigned int>(CSTOPB);
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "unsupported stop bitsn");
        return false;
    }
    /* Set input parity option */
    if (parity != 'n') options.c_iflag |= INPCK;

    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN]  = 0;   /* Update the options and do it NOW */

    options.c_cflag &= ~static_cast<unsigned int>(HUPCL);
    options.c_iflag &= ~static_cast<unsigned int>(INPCK);
    options.c_iflag |= static_cast<unsigned int>(IGNBRK);
    options.c_iflag &= ~static_cast<unsigned int>(ICRNL);
    options.c_iflag &= ~static_cast<unsigned int>(IXON);
    options.c_lflag &= static_cast<unsigned int>(IEXTEN);
    options.c_lflag &= ~static_cast<unsigned int>(ECHOK);
    options.c_lflag &= ~static_cast<unsigned int>(ECHOCTL);
    options.c_lflag &= ~static_cast<unsigned int>(ECHOKE);
    options.c_lflag &= ~static_cast<unsigned int>(ONLCR);
    options.c_oflag = ~static_cast<unsigned int>(ICANON);

    tcflush(mFd, TCIFLUSH);
    if (tcsetattr(mFd, TCSANOW, &options) != 0) {
        perror("setup serial 3");
        return false;
    }
    return true;
}
