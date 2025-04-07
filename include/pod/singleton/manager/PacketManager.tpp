#ifndef PACKETMANAGER_TPP
#define PACKETMANAGER_TPP

#include "PacketManager.hpp"

// 模板函数定义
template <typename T>
std::shared_ptr<T> PacketManager::getPacketById(const int& id) {
    for (const std::shared_ptr<DataPacket>& ptr : mDataPacketPtrs) {
        if (ptr->getId() == id) {
            return std::dynamic_pointer_cast<T>(ptr);
        }
    }

    try {
        throw std::out_of_range("packet not found with id: " + std::to_string(id));
    } catch (const std::out_of_range& e) {
        std::cerr << e.what() << std::endl;
    }

}

#endif  // PACKETMANAGER_TPP
