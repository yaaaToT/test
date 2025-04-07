#ifndef DATAPACKET_HPP
#define DATAPACKET_HPP

#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <opencv2/opencv.hpp>



enum ColorType {
    Red,
    Blue
};

static speed_t speed_arr[] = { B921600, B500000, B460800, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300};
static speed_t name_arr[] = { 921600, 500000, 460800, 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300 };


typedef union {
    float f;
    unsigned char c[4];
} float2uchar;

class DataPacket {
public:
    DataPacket(const int& id);
    static uint8_t crc8Check(uint8_t *addr, int len);

    unsigned char getHead() const;
    unsigned char getTail() const;
    int getId() const;

    virtual void sendData(int& fd);
    virtual void receiveData(int& fd);
    virtual ~DataPacket() = default;
protected:
    unsigned char mHead;
    unsigned char mTail;
    int mGunId;
};

#endif
