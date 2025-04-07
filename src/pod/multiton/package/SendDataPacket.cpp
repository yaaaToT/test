#include "../../../../include/pod/multiton/package/SendDataPacket.hpp"

//#include "../../../../config/packet/0.yml"
// #include <cstring>
// #include <unistd.h>
#include <opencv2/opencv.hpp>

// void SendDataPacket::updateShootMode(SendDataPacket& packet) {
//     if (fabs(packet.pitch.f) >= 1.2) packet.ifRealShoot = 0;
//     else if (packet.ifShoot == 1 && fabs(packet.yaw.f) < 4.0 && fabs(packet.pitch.f) < 1.2 && packet.distance.f >= 0.0 && packet.distance.f < 1.7  ){ packet.ifRealShoot =  1;}
//     else if (packet.ifShoot == 1 && fabs(packet.yaw.f) < 3.5 && fabs(packet.pitch.f) < 1.0 && packet.distance.f >= 1.7 && packet.distance.f < 3.5  ){packet.ifRealShoot =  1;}
//     else if (packet.ifShoot == 1 && fabs(packet.yaw.f) < 2.8 && fabs(packet.pitch.f) < 0.6 && packet.distance.f >= 3.5 && packet.distance.f < 5.0  ){packet.ifRealShoot =  1;}
//     else if (packet.ifShoot == 1 && fabs(packet.yaw.f) < 2.3 && fabs(packet.pitch.f) < 0.4 && packet.distance.f >= 5.0 && packet.distance.f < 10.5 ){packet.ifRealShoot =  1;}
// }

// void SendDataPacket::sendDataPacket(int& fd, std::vector<std::pair<int, SendDataPacket>>& dataPackets) {
//     unsigned char Sdata[18];
//     memset(Sdata,0,sizeof(Sdata));

//     for (const auto & pair : dataPackets) {
//         Sdata[0] = dataPackets.at(pair.first).second.mHead;
//         Sdata[17] = dataPackets.at(pair.first).second.mTail;



//         SendDataPacket packet = pair.second;


//         Sdata[1] = packet.pitch.c[0];
//         Sdata[2] = packet.pitch.c[1];
//         Sdata[3] = packet.pitch.c[2];
//         Sdata[4] = packet.pitch.c[3];
//         // Yaw
//         Sdata[5] = packet.yaw.c[0];
//         Sdata[6] = packet.yaw.c[1];
//         Sdata[7] = packet.yaw.c[2];
//         Sdata[8] = packet.yaw.c[3];
//         // 距离

//         Sdata[9] = packet.distance.c[0];
//         Sdata[10] = packet.distance.c[1];
//         Sdata[11] = packet.distance.c[2];
//         Sdata[12] = packet.distance.c[3];

//         Sdata[13] = packet.id;//装甲板id
//         Sdata[14] = packet.ifShoot;
//         Sdata[15] = packet.ifRealShoot;//ifFirstArmor
//         Sdata[16] = packet.ifFirstArmor;
//     }

//     write(fd, Sdata, 18);
// }

SendDataPacket::SendDataPacket(const int& id) : DataPacket(id) {
    std::string gunIdString = std::to_string(id);

    cv::FileStorage fileReader("../config/packet/"+gunIdString + ".yml",cv::FileStorage::READ);
    fileReader["SendPacket"]["Head"] >> this->mHead;
    fileReader["SendPacket"]["Tail"] >> this->mTail;

    fileReader.release();
}


void SendDataPacket::updateShootMode(std::shared_ptr<SendDataPacket>& packet) {
    if (fabs(packet->pitch.f) >= 1.2) packet->ifRealShoot = 0;
    else if (packet->ifShoot == 1 && fabs(packet->yaw.f) < 4.0 && fabs(packet->pitch.f) < 1.2 && packet->distance.f >= 0.0 && packet->distance.f < 1.7  ){ packet->ifRealShoot =  1;}
    else if (packet->ifShoot == 1 && fabs(packet->yaw.f) < 3.5 && fabs(packet->pitch.f) < 1.0 && packet->distance.f >= 1.7 && packet->distance.f < 3.5  ){packet->ifRealShoot =  1;}
    else if (packet->ifShoot == 1 && fabs(packet->yaw.f) < 2.8 && fabs(packet->pitch.f) < 0.6 && packet->distance.f >= 3.5 && packet->distance.f < 5.0  ){packet->ifRealShoot =  1;}
    else if (packet->ifShoot == 1 && fabs(packet->yaw.f) < 2.3 && fabs(packet->pitch.f) < 0.4 && packet->distance.f >= 5.0 && packet->distance.f < 10.5 ){packet->ifRealShoot =  1;}
}

void SendDataPacket::sendData(int& fd) {
    while (true) {
        

        unsigned char Sdata[18];
        memset(Sdata,0,sizeof(Sdata));

        Sdata[0] = this->mHead;
        Sdata[17] = this->mTail;
        // pitch
        Sdata[1] = this->pitch.c[0];
        Sdata[2] = this->pitch.c[1];
        Sdata[3] = this->pitch.c[2];
        Sdata[4] = this->pitch.c[3];
        // Yaw
        Sdata[5] = this->yaw.c[0];
        Sdata[6] = this->yaw.c[1];
        Sdata[7] = this->yaw.c[2];
        Sdata[8] = this->yaw.c[3];
        // 距离
        Sdata[9] = this->distance.c[0];
        Sdata[10] = this->distance.c[1];
        Sdata[11] = this->distance.c[2];
        Sdata[12] = this->distance.c[3];

        Sdata[13] = this->id;//装甲板id
        Sdata[14] = this->ifShoot;
        Sdata[15] = this->ifRealShoot;//ifFirstArmor
        Sdata[16] = this->ifFirstArmor;
        write(fd, Sdata, 18);
    }
}
