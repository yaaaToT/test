//
// Created by ASUS on 24-11-7.
//

#ifndef RECEIVEDATAPACKET_HPP
#define RECEIVEDATAPACKET_HPP

#include <memory>
#include <vector>
#include "DataPacket.hpp"

class ReceiveDataPacket : public DataPacket{
public:

    ReceiveDataPacket(const int& id);
    void receiveData(int& fd) override;

    static ColorType getColorMode(const std::shared_ptr<ReceiveDataPacket>& packet);
    //static int outpostDestroyed(const std::shared_ptr<ReceiveDataPacket>& packet);
    static int ModeDelection(const std::shared_ptr<ReceiveDataPacket>&packet);

    float2uchar gainPitch;
    float2uchar gainYaw;
    unsigned char color;
    unsigned char mode;
    unsigned char speed;
    unsigned char outpost;

    unsigned char mHead;
    unsigned char mTail;


    // static ColorType getColorMode(const ReceiveDataPacket& packet);
    // static int outpostDestroyed(const ReceiveDataPacket& packet);
    // static void receiveDataPacket(int& fd, std::vector<std::pair<int, ReceiveDataPacket>>& dataPackets);
    // static uint8_t crc8Check(uint8_t *addr, int len);
};



#endif //RECEIVEDATAPACKET_HPP
