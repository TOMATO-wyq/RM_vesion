#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"
#include <Eigen/Dense>

MyExtendedKalmanFilter::MyExtendedKalmanFilter()
{
    state_pre_ = Eigen::Vector<double, 9>::Zero();
    state_post_ = Eigen::Vector<double, 9>::Zero();

    error_cov_pre_ = Eigen::Matrix<double, 9, 9>::Identity();
    error_cov_post_ = Eigen::Matrix<double, 9, 9>::Identity();

    state_transition_matrix_ = Eigen::Matrix<double, 9, 9>::Identity();
    observation_jacobian_ = Eigen::Matrix<double, 4, 9>::Zero();

    process_noise_cov_ = Eigen::Matrix<double, 9, 9>::Zero();
    process_noise_cov_.diagonal() <<
        0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.0005, 0.001, 0.01;
    
    observation_noise_cov_ = Eigen::Matrix<double, 4, 4>::Zero();
    observation_noise_cov_.diagonal() <<
        0.004, 0.004, 0.001, 0.01;

    kalman_gain_ = Eigen::Matrix<double, 9, 4>::Zero();
    origin_observation_ = Eigen::Vector<double, 4>::Zero();
    filtered_observation_ = Eigen::Vector<double, 4>::Zero();
}

void MyExtendedKalmanFilter::initialize(
    const Eigen::Vector<double, 9>& state_pre,
    const Eigen::Matrix<double, 9, 9>& error_cov_pre)
{
    state_pre_ = state_pre;
    state_post_ = state_pre;
    error_cov_pre_ = error_cov_pre;
    error_cov_post_ = error_cov_pre;
}

void MyExtendedKalmanFilter::predict()
{
    state_pre_ = state_transition_matrix_ * state_post_;
    error_cov_pre_ = state_transition_matrix_ * error_cov_post_ * state_transition_matrix_.transpose()
                   + process_noise_cov_;
    
    // 先验 yaw 归一化到 [-π, π]，防止 predict 后角度越界
    while (state_pre_[7] > M_PI) state_pre_[7] -= 2.0 * M_PI;
    while (state_pre_[7] < -M_PI) state_pre_[7] += 2.0 * M_PI;
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::correct(const Eigen::Vector<double, 4>& measurement)
{
    // 保存原始观测值
    origin_observation_ = measurement;

    // 1. 在 state_pre_ 处计算观测雅可比 H
    calculateObservationJacobian();

    // 2. 计算观测预测值 h(x_pre)
    Eigen::Vector<double, 4> predicted_obs;
    measurementFunction(state_pre_, predicted_obs);

    // 3. 计算卡尔曼增益 K = P_pre * H^T * (H * P_pre * H^T + R)^(-1)
    kalman_gain_ = error_cov_pre_ * observation_jacobian_.transpose()
                 * (observation_jacobian_ * error_cov_pre_ * observation_jacobian_.transpose()
                    + observation_noise_cov_).inverse();

    // 4. 计算残差并更新后验状态
    Eigen::Vector<double, 4> residual = measurement - predicted_obs;
    
    // yaw 残差归一化：处理 -π/π 跳变，避免 179° 与 -179° 的差被算成 358°
    while (residual[3] > M_PI) residual[3] -= 2.0 * M_PI;
    while (residual[3] < -M_PI) residual[3] += 2.0 * M_PI;
    
    state_post_ = state_pre_ + kalman_gain_ * residual;

    // 5. 状态值硬约束与角度归一化
    checkValue();

    // 6. 更新后验误差协方差 P_post = (I - K * H) * P_pre
    Eigen::Matrix<double, 9, 9> identity = Eigen::Matrix<double, 9, 9>::Identity();
    error_cov_post_ = (identity - kalman_gain_ * observation_jacobian_) * error_cov_pre_;

    // 7. 计算滤波后的观测值
    measurementFunction(state_post_, filtered_observation_);

    return filtered_observation_;
}

void MyExtendedKalmanFilter::updateStateTransitionMatrix(const double& dt)
{
    /*
        方程如下:  都是线性的
        x^k = x^{k-1} + v^{k-1}_x * dt 
        y^k = y^{k-1} + v^{k-1}_y * dt
        Z^k = Z^{k-1} + v^{k-1}_z * dt
        v^k_x = v^{k-1}_x
        v^k_y = v^{k-1}_y
        v^k_z = v^{k-1}_z 
        r^k = r^{k-1}
        yaw^k = yaw^{k-1} + v^{k-1}_yaw * dt
        v^k_yaw = v^{k-1}_yaw
    */
    state_transition_matrix_ << 
        1, 0, 0, dt, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
}

void MyExtendedKalmanFilter::calculateObservationJacobian()
{
    /*
        方程如下:
        1, 0, 0, 0, 0, 0, sin(yaw), rcos(yaw) , 0,
        0, 1, 0, 0, 0, 0, -cos(yaw), rsin(yaw), 0,
        0, 0, 1, 0, 0, 0, 0,      , 0         , 0,
        0, 0, 0, 0, 0, 0, 0,      , 1         , 0,
    */
    observation_jacobian_ <<
        1, 0, 0, 0, 0, 0, std::sin(state_pre_[7]),  state_pre_[6] * std::cos(state_pre_[7]), 0,
        0, 1, 0, 0, 0, 0, -std::cos(state_pre_[7]), state_pre_[6] * std::sin(state_pre_[7]), 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0;
}

void MyExtendedKalmanFilter::measurementFunction(
    const Eigen::Vector<double, 9>& state,
    Eigen::Vector<double, 4>& observation)
{
    /*  
        方程如下:
        x = x_c + r *sin(yaw)
        y = y_c - r * cos(yaw)
        z = z
        yaw = yaw
    */
    double x_c = state[0];
    double y_c = state[1];
    double z_c = state[2];
    double r = state[6];
    double yaw = state[7];

    observation <<
        x_c + r * std::sin(yaw),
        y_c - r * std::cos(yaw),
        z_c,
        yaw;
}

void MyExtendedKalmanFilter::checkValue()
{
    // r 硬限制在 [0.12, 0.4]
    if (state_post_[6] < 0.12) state_post_[6] = 0.12;
    if (state_post_[6] > 0.4)  state_post_[6] = 0.4;
    
    // yaw 归一化到 [-π, π]
    while (state_post_[7] > M_PI) state_post_[7] -= 2.0 * M_PI;
    while (state_post_[7] < -M_PI) state_post_[7] += 2.0 * M_PI;
}