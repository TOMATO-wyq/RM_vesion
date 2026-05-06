#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "armor_plate_identification/Detector.hpp"

///  @brief 将多个图像拼接显示在一个窗口中（2x2 布局）
///  @param window_name 窗口名称
///  @param images 图像向量，支持 3 或 4 张图
///  @param labels 标签文本（可选）
void showMultiImages(const std::string& window_name, 
                     const std::vector<cv::Mat>& images,
                     const std::vector<std::string>& labels = {});

///  @brief 调试参数控制器，封装 DEBUG_INDENTIFICATION / DEBUG_BASE 公共逻辑
class DebugParamController {
public:
    DebugParamController();

    ///  @brief 处理调试按键（1-6 选参数、T/G 调值、+/- 调播放速度）
    ///  @param key cv::waitKey 返回的按键值
    ///  @param lights 要调节的灯条匹配参数对象
    ///  @param logger ROS2 日志器
    ///  @return true 表示按键已被消费
    bool handleKey(int key, Detector& lights, const rclcpp::Logger& logger);

    ///  @brief 在图像左上角第一行绘制处理用时
    ///  @param process_time_ms 图像从获取到处理完成的用时（毫秒），传负数则不显示
    void drawProcessTime(cv::Mat& img, float process_time_ms);

    ///  @brief 在 process_time 正下方绘制当前播放延迟
    void drawDelay(cv::Mat& img);

    ///  @brief 在图像左上角绘制 6 个可调参数（从第三行开始）
    void drawParams(cv::Mat& img, const Detector& lights);

    ///  @brief 在图像上绘制键盘帮助文字
    ///  @param show_speed_control 是否显示 +/- speed 提示
    void drawDebugInfo(cv::Mat& img, bool show_speed_control);

    ///  @brief 获取当前播放延迟（ms）
    int getPlayDelayMs() const { return play_delay_ms_; }

    ///  @brief 手动设置播放延迟（ms）
    void setPlayDelayMs(int ms) { play_delay_ms_ = std::max(ms, 0); }

private:
    int selected_param_;
    std::vector<std::string> param_names_;
    int play_delay_ms_ = 0;   // 播放延迟，越大越慢
    int x_ = 10;              // 绘制起点 x
    int y_ = 30;              // 绘制起点 y
    int line_h_ = 25;         // 行高
};
