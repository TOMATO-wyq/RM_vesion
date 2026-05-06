#include "armor_plate_identification/DebugIdentifaction.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

void showMultiImages(const std::string& window_name, 
                     const std::vector<cv::Mat>& images,
                     const std::vector<std::string>& labels)
{
    if (images.empty()) return;
    
    // 固定输出窗口大小
    const int grid_cols = 2;  // 2列
    const int grid_rows = 2;  // 2行
    const int cell_width = 640;
    const int cell_height = 480;
    const int text_height = 30;  // 标签高度
    
    // 创建画布（2x2 网格）
    cv::Mat canvas((cell_height + text_height) * grid_rows, 
                   cell_width * grid_cols, 
                   CV_8UC3, cv::Scalar(0, 0, 0));
    
    for (size_t i = 0; i < images.size() && i < 4; ++i) {
        int row = i / grid_cols;
        int col = i % grid_cols;
        
        // 转换单通道为三通道
        cv::Mat img_display;
        if (images[i].channels() == 1) {
            cv::cvtColor(images[i], img_display, cv::COLOR_GRAY2BGR);
        } else {
            img_display = images[i].clone();
        }
        
        // resize 到固定大小
        cv::Mat img_resized;
        cv::resize(img_display, img_resized, cv::Size(cell_width, cell_height));
        
        // 计算放置位置
        int x = col * cell_width;
        int y = row * (cell_height + text_height);
        
        // 复制图像到画布
        cv::Rect roi(x, y + text_height, cell_width, cell_height);
        img_resized.copyTo(canvas(roi));
        
        // 绘制标签
        std::string label = (i < labels.size()) ? labels[i] : ("Image " + std::to_string(i + 1));
        cv::putText(canvas, label, cv::Point(x + 10, y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
    
    cv::imshow(window_name, canvas);
}

/////////////////// DebugParamController //////////////////////////

DebugParamController::DebugParamController()
    : selected_param_(0),
      param_names_({
          "MAX_ANGLE_DIFF",
          "MAX_Y_DIFF_RATIO",
          "MIN_DISTANCE_RATIO",
          "MAX_DISTANCE_RATIO",
          "MIN_LENGTH_RATIO",
          "MIN_X_DIFF_RATIO"
      })
{}

bool DebugParamController::handleKey(int key, Detector& lights, const rclcpp::Logger& logger)
{
    // 1-6：选择参数
    if (key >= '1' && key <= '6') {
        selected_param_ = key - '1';
        RCLCPP_INFO(logger, "选中参数: %s", param_names_[selected_param_].c_str());
        return true;
    }

    // T/G：调节参数值
    bool increase = (key == 't' || key == 'T');
    bool decrease = (key == 'g' || key == 'G');

    if (increase || decrease) {
        float dir = increase ? 1.0f : -1.0f;
        switch (selected_param_) {
            case 0: // MAX_ANGLE_DIFF
                lights.MAX_ANGLE_DIFF = std::clamp(lights.MAX_ANGLE_DIFF + dir * 0.5f, 0.0f, 30.0f);
                break;
            case 1: // MAX_Y_DIFF_RATIO
                lights.MAX_Y_DIFF_RATIO = std::max(lights.MAX_Y_DIFF_RATIO + dir * 0.05f, 0.0f);
                break;
            case 2: // MIN_DISTANCE_RATIO
                lights.MIN_DISTANCE_RATIO = std::max(lights.MIN_DISTANCE_RATIO + dir * 0.1f, 0.0f);
                break;
            case 3: // MAX_DISTANCE_RATIO
                lights.MAX_DISTANCE_RATIO = std::max(lights.MAX_DISTANCE_RATIO + dir * 0.1f, 0.0f);
                break;
            case 4: // MIN_LENGTH_RATIO
                lights.MIN_LENGTH_RATIO = std::clamp(lights.MIN_LENGTH_RATIO + dir * 0.05f, 0.0f, 1.0f);
                break;
            case 5: // MIN_X_DIFF_RATIO
                lights.MIN_X_DIFF_RATIO = std::max(lights.MIN_X_DIFF_RATIO + dir * 0.05f, 0.0f);
                break;
        }
        return true;
    }

    // +/-：速度控制
    if (key == '+' || key == '=') {
        play_delay_ms_ = std::max(play_delay_ms_ - 10, 0);
        RCLCPP_INFO(logger, "播放延迟: %d ms", play_delay_ms_);
        return true;
    }
    if (key == '-' || key == '_') {
        play_delay_ms_ += 10;
        RCLCPP_INFO(logger, "播放延迟: %d ms", play_delay_ms_);
        return true;
    }

    return false;
}

void DebugParamController::drawProcessTime(cv::Mat& img, float process_time_ms)
{
    if (process_time_ms < 0.0f) return;
    std::string time_text = "Process: " + std::to_string(process_time_ms) + " ms";
    time_text = time_text.substr(0, time_text.find('.') + 3);
    cv::putText(img, time_text, cv::Point(x_, y_ + 0 * line_h_),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
}

void DebugParamController::drawDelay(cv::Mat& img)
{
    std::string delay_text = "Delay: " + std::to_string(play_delay_ms_) + " ms";
    cv::putText(img, delay_text, cv::Point(x_, y_ + 1 * line_h_),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
}

void DebugParamController::drawParams(cv::Mat& img, const Detector& lights)
{
    // 下面6行：可调参数，放在第三行之后
    for (int i = 0; i < 6; ++i) {
        float val = 0.0f;
        switch (i) {
            case 0: val = lights.MAX_ANGLE_DIFF; break;
            case 1: val = lights.MAX_Y_DIFF_RATIO; break;
            case 2: val = lights.MIN_DISTANCE_RATIO; break;
            case 3: val = lights.MAX_DISTANCE_RATIO; break;
            case 4: val = lights.MIN_LENGTH_RATIO; break;
            case 5: val = lights.MIN_X_DIFF_RATIO; break;
        }
        std::string text = param_names_[i] + ": " + std::to_string(val);
        text = text.substr(0, text.find('.') + 3);
        cv::Scalar color = (i == selected_param_) ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 255);
        cv::putText(img, text, cv::Point(x_, y_ + (i + 2) * line_h_),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }
}

void DebugParamController::drawDebugInfo(cv::Mat& img, bool show_speed_control)
{
    int base_row = 8;  // 1 行 process_time + 1 行 delay + 6 个参数
    if (show_speed_control) {
        cv::putText(img, "1-6:select  T/G:adj  +/-:speed  P:pause  ESC:exit",
                    cv::Point(x_, y_ + base_row * line_h_ + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    } else {
        cv::putText(img, "1-6:select  T/G:adj  P:pause  ESC:exit",
                    cv::Point(x_, y_ + base_row * line_h_ + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}
