
#include "../../../include/module/inference/YOLO11module.hpp"
// Yolo11Module 类的构造函数，用于初始化模型路径、输入形状、类别文件和是否使用CUDA
Yolo11Module::Yolo11Module(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda)
    : modelPath(onnxModelPath),
    modelShape(modelInputShape),
    classesPath(classesTxtFile),
    cudaEnabled(runWithCuda){
    net = cv::dnn::readNetFromONNX(modelPath);
    loadClassesFromFile(); // 加载类别文件
}
Yolo11Module inf("../src/module/inference/buffbest.onnx", cv::Size(256, 256), "../src/module/inference/classes.txt", false);
// runInference 函数，用于对输入图像进行推理，并返回检测结果
Detection Yolo11Module::runInference(const cv::Mat &input) {
    cv::Mat modelInput = input,blob;
    // 如果需要将图像转换为正方形，则调用 formatToSquare 函数
    if(letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput);
    // 创建一个 blob 用于神经网络的输入
    cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    // 获取神经网络的输出
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    int rows = outputs[0].size[1];
    int dimensions = outputs[0].size[2];
    bool yolov11 = false;
    if (dimensions > rows) {
        yolov11 = true;
        rows = outputs[0].size[2];
        dimensions = outputs[0].size[1];
        outputs[0] = outputs[0].reshape(1, dimensions);
        transpose(outputs[0], outputs[0]);
    }
    float *data = (float *)outputs[0].data;
    int class_ids;
    float confidence;
    cv::Point class_id;
    double maxClassScore;
    float *classes_scores;
    // 遍历每个检测框
    for (int i = 0; i < rows; ++i) {
        if (yolov11) {
            classes_scores = data + 4;
            cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);
            if (maxClassScore > modelScoreThreshold) {
                confidence=maxClassScore;
                class_ids=class_id.x;
            }
        }
        else {
            confidence = data[4];
            if (confidence >= 0.7) {
                float *classes_scores = data + 5;
                cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                double max_class_score;
                minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
                if (max_class_score < 0.7) continue;
            }
        }
        data += dimensions;
    }
    // 构建检测结果
    Detection result;
    result.class_id = class_ids;
    result.confidence = confidence;
    if(result.class_id>14)
        result.className="None";
    else
    result.className = classes[result.class_id];
    return result;
}
// loadClassesFromFile 函数，用于从文件中加载类别列表
void Yolo11Module::loadClassesFromFile() {
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open()) {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}
cv::Mat Yolo11Module::formatToSquare(const cv::Mat &source){
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}
// run 函数，用于运行整个模块
bool Yolo11Module::run(const cv::Mat& src) {
    detection = inf.runInference(src);
    if(detection.className=="None")
        return false;
    return true;
}
//调用函数
std::string Yolo11Module::GetArmorName(){
    return detection.className;
}
float Yolo11Module::GetArmorConfidence(){
    return detection.confidence;
}
bool Yolo11Module::GetArmorType(){
    if(detection.className=="BA")     
        return true;
    return false;
}
