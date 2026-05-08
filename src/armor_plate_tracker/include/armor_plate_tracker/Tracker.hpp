#pragma once
#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"

#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

struct AngleRecord {
    builtin_interfaces::msg::Time stamp;
    // 单位是弧度
    float yaw_abs;
    float pitch_abs;
};


struct TrackingOverlayData {
    bool has_result = false;
    bool is_lost = false;
    Eigen::Vector3d measured_position;
    float measured_yaw = 0.0f;
    float measured_pitch = 0.0f;
    float filter_yaw = 0.0f;
    float filter_pitch = 0.0f;
    float distance = 0.0f;
};
// opencv坐标系转换为一个云台的坐标系, x向前，y向左，z向上
extern const Eigen::Matrix3d R_w_cv;

class Tracker
{
private:
    // EKF 滤波器
    MyExtendedKalmanFilter ekf_;
    
    // 当前滤波结果 -- 相机坐标系(增量角)
    float yaw_;
    float pitch_;
    // 绝对角度
    float yaw_abs_;
    float pitch_abs_;
    Eigen::Matrix3d R_w_c_; // R_{w<-c}
    Eigen::Matrix3d R_c_w_; // R_{c<-w}
    Eigen::Quaterniond q_w_c_;
    // 时间相关
    double last_update_time_;   // 上次更新时间（秒）
    double last_detection_time_;// 上次检测到目标的时间（秒）
    // 跟踪状态
    bool initialized_;          // 是否已初始化
    double max_lost_time_;      // 最大允许丢失时间（秒），默认0.5s
    bool is_lost_;              // 是否处于丢失状态
    // 突变检测阈值（度）
    float yaw_mutation_threshold_;
    float last_armor_pose_yaw_world_;
    std::string last_armor_number_;
    // 当前帧选中的原始测量值（相机系）
    float measured_yaw_;
    float measured_pitch_;
    // 获得世界坐标系下的点
    Eigen::Vector3d measured_position_world_;
    Eigen::Vector3d filter_position_world_;
    Eigen::Vector3d center_point_world_;
    Eigen::Vector3d center_velocity_;
    // 获得相机坐标系下的点
    Eigen::Vector3d measured_position_camera_;
    Eigen::Vector3d filter_position_camera_;
    Eigen::Quaterniond measured_orientation_world_;
    Eigen::Quaterniond filter_orientation_world_;
    
    // 选择最佳匹配目标
    void selectBestMatch(const std::vector<ArmorPlate>& armor_plates, ArmorPlate& target_armor);
    
    // 检查是否突变
    bool isYawMutation(const float& armor_pose_yaw);
    
    // 检查是否丢失太久
    bool isLostTooLong(double current_time) const;

    // 更新测量值相关成员变量
    void updateMeasurement(const ArmorPlate& armor_plate, double current_time);

    // 更新滤波值相关成员变量（位置+角度）
    void updateFilteredValue(const Eigen::Vector3d& pc_f, const Eigen::Vector3d& pw_f);
public:
    // 默认构造函数
    Tracker();
    
    // 重置滤波器（无目标时调用）
    void reset();
    
    // 初始化 EKF（检测到第一个目标时调用）
    void init(const ArmorPlate& armor_plate, double current_time);
    
    // 设置最大丢失时间（秒）
    void setMaxLostTime(double seconds) { max_lost_time_ = seconds; }
    
    // 设置突变阈值
    void setMutationThreshold(float yaw_thresh) { yaw_mutation_threshold_ = yaw_thresh; }
    
    
    void Update(const std::vector<ArmorPlate>& armor_plates,
                double current_time,
                float yaw_abs, float pitch_abs
    );
    
    // 获取滤波后的值
    float getYaw() const { return yaw_; }
    float getPitch() const { return pitch_; }
    // 获取原始测量值（选中目标，相机系）
    float getMeasuredYaw() const { return measured_yaw_; }
    float getMeasuredPitch() const { return measured_pitch_; }
    Eigen::Vector3d getMeasuredPosition() const { return measured_position_camera_; }

    // 获取是否丢失目标
    bool isLost() const { return is_lost_; }
    // 获取是否已初始化
    bool isInitialized() const { return initialized_; }
    // 获取上次更新时间
    double getLastUpdateTime() const { return last_update_time_; }

    // 获得世界坐标系下的点
    Eigen::Vector3d getMeasuredPositionWorld() const { return measured_position_world_; }
    Eigen::Vector3d getFilterPositionWorld() const { return filter_position_world_; }
    Eigen::Quaterniond getMeasuredOrientationWorld() const { return measured_orientation_world_; }
    Eigen::Quaterniond getFilterOrientationWorld() const { return filter_orientation_world_; }
    Eigen::Vector3d getCenterPointWorld() const { return center_point_world_; }
    Eigen::Vector3d getCenterVelocity() const { return center_velocity_; }
    // 获得相机坐标系下的点 
    Eigen::Vector3d getMeasuredPositionCamera() const { return measured_position_camera_; }
    Eigen::Vector3d getFilterPositionCamera() const { return filter_position_camera_; }
};

// 工具函数
float normalizeRadAngle(float rad);
float calculatePoseYaw(const Eigen::Quaterniond &q);
float calculateYaw(const Eigen::Vector3d& tvec);
float calculatePitch(const Eigen::Vector3d& tvec);
float calculateDistance(const Eigen::Vector3d& tvec);
Eigen::Quaterniond getQuaternionFromYaw(float yaw);