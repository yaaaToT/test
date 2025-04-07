#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
// #include <fstream>
#include <vector>
#include <string>
// #include <random>
// OpenCV / DNN / Inference
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
struct Detection{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
};
class Yolo11Module{
public:
    Yolo11Module()=default;
    Yolo11Module(const std::string &onnxModelPath, const cv::Size &modelInputShape = {256, 256}, const std::string &classesTxtFile = "", const bool &runWithCuda = false);
    Detection runInference(const cv::Mat &Input);
    bool run(const cv::Mat& src);
    std::string GetArmorName();
    bool GetArmorType();
    float GetArmorConfidence();
private:
    void loadClassesFromFile();
    cv::Mat formatToSquare(const cv::Mat &source);
    std::string modelPath;
    std::string classesPath;
    bool cudaEnabled{};
    std::vector<std::string> classes{
};
    cv::Size modelShape{};
    float modelConfidenceThreshold{0.7};
    float modelScoreThreshold{0.45};
    float modelNMSThreshold{0.50};
    bool letterBoxForSquare{true};
    cv::dnn::Net net;
    Detection detection ;
};

#endif // INFERENCE_H
