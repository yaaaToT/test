//
// Created by torosamy on 24-11-19.
//

#include "../../../../include/pod/singleton/manager/PacketManager.hpp"

#include "../../../../include/pod/singleton/SerialPort.hpp"


void PacketManager::addPacket(const std::shared_ptr<DataPacket>& packetPtr) {
    mDataPacketPtrs.push_back(packetPtr);
}

void PacketManager::loadSendTasks(std::vector<std::thread>& tasks) {
    for (const std::shared_ptr<DataPacket>& packetPtr : mDataPacketPtrs) {
        tasks.emplace_back([packetPtr] { packetPtr->sendData(SerialPort::mFd); });
    }
}

void PacketManager::loadReceiveTasks(std::vector<std::thread>& tasks) {
    for (const std::shared_ptr<DataPacket>& packetPtr : mDataPacketPtrs) {
        tasks.emplace_back([packetPtr] { packetPtr->receiveData(SerialPort::mFd); });
    }
}

int PacketManager::getSize() const {
    return mDataPacketPtrs.size();
}
