#ifndef PACKETMANAGER_HPP
#define PACKETMANAGER_HPP
#include "../../multiton/package/DataPacket.hpp"
#include <memory>
#include <thread>
class PacketManager {
public:
    PacketManager() = default;
    void addPacket(const std::shared_ptr<DataPacket>& packetPtr);
    void loadSendTasks(std::vector<std::thread>& tasks);
    void loadReceiveTasks(std::vector<std::thread>& tasks);
    int getSize() const;

    template <typename T>
    std::shared_ptr<T> getPacketById(const int& id);
private:
    std::vector<std::shared_ptr<DataPacket>> mDataPacketPtrs;
};

#include "PacketManager.tpp"


#endif //PACKETMANAGER_HPP
