#include "armor_plate_tracker/Tracker.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "rclcpp/rclcpp.hpp"

// opencv坐标系 → 云台坐标系 (x向前, y向左, z向上)
const Eigen::Matrix3d R_w_cv = (Eigen::Matrix3d() <<
    0.0 ,  0.0,  1.0,
    -1.0,  0.0,  0.0,
    0.0 , -1.0,  0.0).finished();

// 默认构造函数
Tracker::Tracker()
    : yaw_(0.0f), pitch_(0.0f)
    , last_update_time_(0.0)
    , last_detection_time_(0.0)
    , initialized_(false)
    , max_lost_time_(0.1)
    , is_lost_(true)
    , yaw_mutation_threshold_(0.05f)
    , last_armor_number_("")
{
}
void Tracker::reset()
{
    // EKF 构造函数已设置 Q/R，只需重置状态和协方差
    Eigen::Vector<double, 9> zero_state = Eigen::Vector<double, 9>::Zero();
    Eigen::Matrix<double, 9, 9> identity_P = Eigen::Matrix<double, 9, 9>::Identity();
    ekf_.initialize(zero_state, identity_P);
    
    initialized_ = false;
    last_update_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_armor_pose_yaw_world_ = 0.0f;
    yaw_ = 0.0f;
    pitch_ = 0.0f;
    measured_yaw_ = 0.0f;
    measured_pitch_ = 0.0f;
    last_armor_number_ = "";
}

void Tracker::init(const ArmorPlate& armor_plate, double current_time)
{
    // 跟新滤波器
    Eigen::Vector3d target_position(
        armor_plate.pose.position.x,
        armor_plate.pose.position.y,
        armor_plate.pose.position.z
    );
    Eigen::Quaterniond target_plate_q(
        armor_plate.pose.orientation.w,
        armor_plate.pose.orientation.x,
        armor_plate.pose.orientation.y,
        armor_plate.pose.orientation.z
    );
    Eigen::Quaterniond qw_m = q_w_c_ * target_plate_q;
    float armor_pose_yaw_world = calculatePoseYaw(qw_m);

    Eigen::Vector3d pw = R_w_c_ * target_position;
    const double r_init = 0.26;
    double x_c0 = pw.x() - r_init * std::sin(armor_pose_yaw_world);
    double y_c0 = pw.y() - r_init * std::cos(armor_pose_yaw_world);
    
    Eigen::Vector<double, 9> init_state;
    init_state << x_c0, y_c0, pw.z(), 0.0, 0.0, 0.0, r_init, armor_pose_yaw_world, 0.0;
    
    Eigen::Matrix<double, 9, 9> init_P = Eigen::Matrix<double, 9, 9>::Identity();
    init_P.diagonal() << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 0.01, 0.1, 1.0;
    
    ekf_.initialize(init_state, init_P);

    // 首次初始化：测量值 = 滤波值 = 初始观测
    updateMeasurement(armor_plate, current_time);
    updateFilteredValue(target_position, pw);

    initialized_ = true;
}

// 选择最佳匹配目标
void Tracker::selectBestMatch(const std::vector<ArmorPlate>& armor_plates, ArmorPlate& target_armor) 
{
    size_t num_plates = armor_plates.size();
    // 如果只有一个目标，直接选择
    if (num_plates == 1) {
        target_armor = armor_plates[0];
        return;
    }
    // 多个目标
    if (!initialized_) {
        // 未初始化时，选择 image_distance_to_center 最小的目标
        size_t min_idx = 0;
        float min_dist = armor_plates[0].image_distance_to_center;
        for (size_t i = 1; i < num_plates; i++) {
            float image_distance  = armor_plates[i].image_distance_to_center;
            if (image_distance < min_dist) {
                min_dist = image_distance;
                min_idx = i;
            }
        }
        // 保存结果
        target_armor = armor_plates[min_idx];
        return;
    } 
    // 选择与当前预测最接近的

    // 使用 EKF 当前状态计算预测的装甲板中心位置
    Eigen::Vector<double, 9> state = ekf_.getStatePost();
    double r = state[6];
    double yaw = state[7];
    Eigen::Vector3d pred_pos(
        state[0] + r * std::sin(yaw),
        state[1] - r * std::cos(yaw),
        state[2]
    );
    float min_dist = std::numeric_limits<float>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < num_plates; ++i) {
        Eigen::Vector3d p_c;
        p_c[0] = armor_plates[i].pose.position.x;
        p_c[1] = armor_plates[i].pose.position.y;
        p_c[2] = armor_plates[i].pose.position.z;
        Eigen::Vector3d p_w = R_w_c_ * p_c;
        Eigen::Vector3d diff = p_w - pred_pos;
        float dist_sq = diff.norm();
        if (dist_sq < min_dist) {
            min_dist = dist_sq;
            best_idx = i;
        }
    }
    // 保存结果
    target_armor = armor_plates[best_idx];
}

// 检查Yaw是否突变
bool Tracker::isYawMutation(const float& armor_pose_yaw)
{
    if (!initialized_) {
        return false;
    }
    float dy = std::abs(armor_pose_yaw - last_armor_pose_yaw_world_);
    dy = normalizeRadAngle(dy);
    return (dy > yaw_mutation_threshold_);
}

// 检查是否丢失太久
bool Tracker::isLostTooLong(double current_time) const
{
    double lost_duration = current_time - last_detection_time_;
    return lost_duration > max_lost_time_;
}

// 主更新函数
void Tracker::Update(const std::vector<ArmorPlate>& armor_plates,
                     double current_time,
                     float yaw_abs, float pitch_abs)
{
    // 计算dt（与上次更新的时间差）
    double dt = 0.01;  // 默认10ms
    if (last_update_time_ > 0.0) {
        dt = current_time - last_update_time_;
        if (dt <= 0.0) dt = 0.001;  // 最小1ms
        if (dt > 1.0) dt = 1.0;     // 最大1s
    }
    last_update_time_ = current_time;
    
    // 坐标系转换计算
    double psi = yaw_abs;
    double theta = pitch_abs;
    // Rw<-c = Rx(pitch_abs)Ry(-yaw_abs) 解耦矩阵
    Eigen::Matrix3d R_cv_c;
    R_cv_c << cos(psi),            0,            -sin(psi),
            -sin(theta)*sin(psi), cos(theta),   -sin(theta)*cos(psi),
            cos(theta)*sin(psi),  sin(theta),   cos(theta)*cos(psi);
    R_w_c_ = R_w_cv * R_cv_c;
    R_c_w_ = R_w_c_.transpose();
    q_w_c_ = Eigen::Quaterniond(R_w_c_);
    // 更新状态转移矩阵中的dt
    ekf_.updateStateTransitionMatrix(dt);
    // 没有目标情况
    if (armor_plates.size() == 0) {
        // 没有检测到目标，进入丢失状态
        is_lost_ = true;
        if (isLostTooLong(current_time)) reset(); // 丢失时间过长，重置
        else if (initialized_) ekf_.predict(); // 丢失但未过长，继续预测
        return;
    }
    // 有检测结果，选择最佳匹配
    ArmorPlate target_plate;
    selectBestMatch(armor_plates, target_plate);
    
    // 提取选中的目标位置（相机系）
    Eigen::Vector3d target_position(
        target_plate.pose.position.x,
        target_plate.pose.position.y,
        target_plate.pose.position.z
    );
    
    // 提取装甲板姿态四元数（相机系）
    Eigen::Quaterniond target_plate_q(
        target_plate.pose.orientation.w,
        target_plate.pose.orientation.x,
        target_plate.pose.orientation.y,
        target_plate.pose.orientation.z
    );
    // 提前计算世界坐标系下的装甲板姿态四元数
    Eigen::Quaterniond qw_m = q_w_c_ * target_plate_q;
    
    // 计算装甲板姿态 yaw（世界坐标系）
    float armor_pose_yaw_world = calculatePoseYaw(qw_m);
    // 判断突变
    if (isYawMutation(armor_pose_yaw_world)) reset();
    // 如果是第一次初始化，设置初始状态（世界坐标系）
    if (!initialized_) {
        init(target_plate, current_time);
        return;
    }

    // 保存当前帧选中的原始测量值
    updateMeasurement(target_plate, current_time);

    // 世界坐标系下 EKF: Predict
    ekf_.predict();

    // 世界坐标系下的原始测量值
    Eigen::Vector3d pw_m = R_w_c_ * target_position;
    Eigen::Vector<double, 4> measurement;
    measurement << pw_m.x(), pw_m.y(), pw_m.z(), armor_pose_yaw_world;

    Eigen::Vector<double, 9> state;
    ekf_.correct(measurement);
    state = ekf_.getStatePost();

    Eigen::Vector3d pw_f = ekf_.getFilteredObservation().head<3>();
    Eigen::Vector3d pc_f = R_c_w_ * pw_f;
    updateFilteredValue(pc_f, pw_f);

    // EKF 状态向量提取
    center_point_world_ = Eigen::Vector3d(state[0], state[1], state[2]);
    center_velocity_    = Eigen::Vector3d(state[3], state[4], 0);
    filter_orientation_world_ = getQuaternionFromYaw(state[7]);
    //// DEBUG //////
    RCLCPP_INFO(rclcpp::get_logger("TRACKER_DEBUG"), "EKF State: x:%.4f y:%.4f z:%.4f r:%.4f yaw:%.4f",
        state[0], state[1], state[2], state[6], state[7]);
    RCLCPP_INFO(rclcpp::get_logger("TRACKER_DEBUG"), "EKF Measurement: x_a:%.4f y_a: %.4f z_a:%.4f yaw:%.4f",
        measurement[0], measurement[1], measurement[2], measurement[3]
    );
    //RCLCPP_INFO(rclcpp::get_logger("TRACKER_DEBUG"), "LAST ARMOR: %s", last_armor_number_.c_str());
}
float calculatePoseYaw(const Eigen::Quaterniond &q)
{
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
} 
float calculateYaw(const Eigen::Vector3d& tvec)
{
    return -static_cast<float>(std::atan2(tvec.x(), tvec.z()));
}
float calculatePitch(const Eigen::Vector3d& tvec)
{
    double horizontal_dist = std::sqrt(tvec.x() * tvec.x() + tvec.z() * tvec.z());
    return static_cast<float>(std::atan2(-tvec.y(), horizontal_dist));
}
float calculateDistance(const Eigen::Vector3d& tvec)
{
    return static_cast<float>(tvec.norm());
}
float normalizeRadAngle(float rad)
{
    while(rad > M_PI) rad -= 2.0 * M_PI;
    while(rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}
Eigen::Quaterniond getQuaternionFromYaw(float yaw)
{
    // 有关世界Z轴的旋转
    Eigen::AngleAxisd yawAngle(yaw + M_PI / 2, Eigen::Vector3d::UnitZ());
    return Eigen::Quaterniond(yawAngle);
}

void Tracker::updateMeasurement(const ArmorPlate& armor_plate, double current_time)
{
    Eigen::Vector3d pc(
        armor_plate.pose.position.x,
        armor_plate.pose.position.y,
        armor_plate.pose.position.z
    );
    Eigen::Vector3d pw = R_w_c_ * pc;

    measured_position_camera_ = pc;
    measured_position_world_  = pw;
    measured_yaw_   = calculateYaw(pc);
    measured_pitch_ = calculatePitch(pc);

    Eigen::Quaterniond qc(
        armor_plate.pose.orientation.w,
        armor_plate.pose.orientation.x,
        armor_plate.pose.orientation.y,
        armor_plate.pose.orientation.z
    );
    Eigen::Quaterniond qw = q_w_c_ * qc;
    float armor_yaw_world = calculatePoseYaw(qw);
    measured_orientation_world_ = getQuaternionFromYaw(armor_yaw_world);

    last_armor_pose_yaw_world_ = armor_yaw_world;
    last_armor_number_ = armor_plate.number;
    last_detection_time_ = current_time;
    is_lost_ = false;
}

void Tracker::updateFilteredValue(const Eigen::Vector3d& pc_f, const Eigen::Vector3d& pw_f)
{
    filter_position_camera_ = pc_f;
    filter_position_world_  = pw_f;
    yaw_   = calculateYaw(pc_f);
    pitch_ = calculatePitch(pc_f);
}