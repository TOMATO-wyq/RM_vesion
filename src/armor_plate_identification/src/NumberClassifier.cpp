#include "armor_plate_identification/NumberClassifier.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <fstream>
#include <stdexcept>

NumberClassifier::NumberClassifier(
    const std::string& model_path,
    const std::string& label_path,
    double threshold,
    const std::vector<std::string>& ignore_classes
) : ignore_classes_(ignore_classes), threshold_(threshold)
{
    // 加载模型
    net_ = cv::dnn::readNetFromONNX(model_path);
    if (net_.empty()) {
        throw std::runtime_error("Failed to load ONNX model: " + model_path);
    }
    // 加载标签
    std::ifstream label_file(label_path);
    if (!label_file.is_open()) {
        throw std::runtime_error("Failed to open label file: " + label_path);
    }
    std::string line;
    while(std::getline(label_file, line)) {
        if (!line.empty()) {
            class_names_.push_back(line);
        }
    }
    if (class_names_.empty()) {
        throw std::runtime_error("Label file is empty: " + label_path);
    }
}

void NumberClassifier::classify(std::vector<Armor>& armors)
{
    if (net_.empty() || class_names_.empty()) {
        throw std::runtime_error("NumberClassifier is not initialized");
    }
    // 推理
    for (auto& armor : armors) {
        // 如果号码ROI为空，直接跳过
        if(armor.number_roi_.empty()) {
            armor.confidence_ = 0.0;
            continue;
        }
        cv::Mat image = armor.number_roi_;
        // 打包成blob，同时做归一化 (1/255) 和尺寸校验
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(20, 28), 0, false, false);
        // 把blob输入网络
        net_.setInput(blob);
        // 前向传播
        cv::Mat outputs = net_.forward();
        if (outputs.empty()) {
            armor.confidence_ = 0.0;
            armor.number_.clear();
            continue;
        }
        // 取最大值和索引
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        if (sum <= 1e-6f) {
            armor.confidence_ = 0.0;
            armor.number_.clear();
            continue;
        }
        softmax_prob /= sum;
        // 取最大概率的索引
        double confidence;
        cv::Point class_id_point;
        cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;
        if (label_id < 0 || static_cast<size_t>(label_id) >= class_names_.size()) {
            armor.confidence_ = 0.0;
            armor.number_.clear();
            continue;
        }
        // 填入装甲板信息
        armor.confidence_ = confidence;
        armor.number_ = class_names_[label_id];
    }
    // 过滤掉需要忽略的类别或者置信度过低的类别
    armors.erase(std::remove_if(armors.begin(), armors.end(), 
        [this](const Armor& armor) {
            // 置信度太低
            if (armor.confidence_ < threshold_) return true;
            // 需要忽略的类别
            for (const auto& ignore_class : ignore_classes_) {
                if (armor.number_ == ignore_class) return true;
            }
            // 其他情况保留
            return false;
        }
    ), armors.end());
}

void drawNumberTest(cv::Mat& img, const Armor& armor)
{
    if (armor.number_.empty()) return;
    // 在上边上面画出号码和置信度
    std::string text = armor.number_ + " (" + std::to_string(static_cast<int>(armor.confidence_ * 100)) + "%)";
    cv::putText(img, text, (armor.points_[0] + armor.points_[1]) / 2, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
}

void drawAllNumberTest(cv::Mat& img, const std::vector<Armor>& armors)
{
    for (const auto& armor : armors) {
        drawNumberTest(img, armor);
    }
}
