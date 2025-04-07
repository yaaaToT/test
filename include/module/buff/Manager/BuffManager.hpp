#ifndef BUFFMANAGER_HPP
#define BUFFMANAGER_HPP

#include "../Detector/BuffArmor.hpp"
#include "../Detector/Detector.hpp"
#include "opencv2/opencv.hpp"

class BuffManager
{
public:
    BuffManager();

    bool updateTarget();

};

#endif //BUFFMANAGER_HPP
