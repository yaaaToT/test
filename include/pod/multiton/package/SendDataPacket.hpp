//
// Created by ASUS on 24-11-7.
//

#ifndef SENDDATAPACKET_HPP
#define SENDDATAPACKET_HPP

#include <vector>
#include "DataPacket.hpp"

class SendDataPacket : public DataPacket {
public:
    SendDataPacket(const int& id);
    void sendData(int &fd) override;
    static void updateShootMode(std::shared_ptr<SendDataPacket>& packet);

    float2uchar pitch;
    float2uchar yaw;
    float2uchar distance;
    float2uchar x;
    float2uchar y;
    float2uchar z;
    unsigned char id;
    unsigned char ifShoot=0;
    unsigned char ifRealShoot=0;
    unsigned char ifFirstArmor=0;

    unsigned char mHead;
    unsigned char mTail;

    // static void updateShootMode(SendDataPacket& packet);

    // static void sendDataPacket(int& fd, std::vector<std::pair<int, SendDataPacket>>& dataPackets);

};




#endif //SENDDATAPACKET_HPP
