#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_tracker/DebugTracker.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include "armor_plate_interfaces/msg/tracker_data.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <mutex>
#include <deque>

using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::GimbalAngle;
using armor_plate_interfaces::msg::TrackerData;
using armor_plate_interfaces::msg::TrackerDebug;

class ArmorPlateTracker : public rclcpp::Node
{
private:
    // ===== 装甲板跟踪器  ===== //
    Tracker tracker_;
    double max_lost_time_;
    double mutation_yaw_threshold_;
    // ===== ROS 相关  ===== //
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ArmorPlates>::SharedPtr armor_plates_sub_;
    rclcpp::Subscription<GimbalAngle>::SharedPtr gimbal_ganle_sub_;
    rclcpp::Publisher<AimCommand>::SharedPtr aim_command_pub_;
    rclcpp::Publisher<TrackerData>::SharedPtr tracker_data_pub_;
    // 数据可视化
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    // ===== 时间相关 ===== //
    double current_time_ = 0.0;
    builtin_interfaces::msg::Time image_stamp_;
    // ===== 绝对角度 ===== //
    std::deque<AngleRecord> angle_buffer_;
    std::mutex angle_buffer_mutex_;

    // ===== DEBUG =====//
    bool debug_;
    rclcpp::Publisher<TrackerDebug>::SharedPtr tracker_debug_pub_;

    void init()
    {
        // ===== 参数获取 ===== //
        max_lost_time_ = this->declare_parameter<double>("max_lost_time", 0.5);
        mutation_yaw_threshold_ = this->declare_parameter<double>("mutation_yaw_threshold", 3.0);
        // ===== ROS 相关 ===== //
        armor_plates_sub_ = this->create_subscription<ArmorPlates>(
            "armor_plates", 
            10,
            std::bind(&ArmorPlateTracker::ArmorPlatesCallBack, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ArmorPlateTracker::info, this));
        aim_command_pub_ = this->create_publisher<AimCommand>("aim_command", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        gimbal_ganle_sub_ = this->create_subscription<GimbalAngle>(
            "gimbal_angle", 10,
            [this](const GimbalAngle::SharedPtr msg){
                std::lock_guard<std::mutex> lock(angle_buffer_mutex_);
                angle_buffer_.push_back({msg->stamp, msg->yaw_abs, msg->pitch_abs});
                if(angle_buffer_.size() > 50) angle_buffer_.pop_front();
            }
        );
        tracker_data_pub_ = this->create_publisher<TrackerData>("tracker_data", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        // ===== DEBUG ===== //
        debug_ = this->declare_parameter<bool>("debug", false);
        if(debug_) {
            tracker_debug_pub_ = this->create_publisher<TrackerDebug>("tracker_debug", 10);
        }
        // ===== 装甲板跟踪器 ===== //
        tracker_.setMaxLostTime(max_lost_time_);
        tracker_.setMutationThreshold(mutation_yaw_threshold_);
        tracker_.reset();
        
        if (debug_) RCLCPP_INFO(this->get_logger(), "启动DEBUG模式");
    }
    double findClosestAngle(const builtin_interfaces::msg::Time& image_stamp, float& output_yaw, float& output_pitch)
    {
        double image_time = image_stamp.sec + image_stamp.nanosec * 1e-9;
        
        // 获取角度缓存
        std::vector<AngleRecord> local_buffer;
        {
            std::lock_guard<std::mutex> lock(angle_buffer_mutex_);
            if (angle_buffer_.empty()) {
                output_yaw = 0.0f;
                output_pitch = 0.0f;
                // RCLCPP_WARN(this->get_logger(), "角度缓存为空，无法对齐，使用 0");
                return 0.0;
            }
            local_buffer.assign(angle_buffer_.begin(), angle_buffer_.end());
        }
        
        auto closest_it = local_buffer.begin();
        double min_diff = std::abs(
            (closest_it->stamp.sec + closest_it->stamp.nanosec * 1e-9) - image_time);
        
        for (auto it = local_buffer.begin() + 1; it != local_buffer.end(); ++it) {
            double record_time = it->stamp.sec + it->stamp.nanosec * 1e-9;
            double diff = std::abs(record_time - image_time);
            if (diff < min_diff) {
                min_diff = diff;
                closest_it = it;
            }
        }
        
        output_yaw = closest_it->yaw_abs;
        output_pitch = closest_it->pitch_abs;
        
        if (min_diff > 0.05) {
            RCLCPP_WARN(this->get_logger(), 
                "角度-图像时间差 %.1f ms，对齐可能不准", min_diff * 1000.0);
        }
        return closest_it->stamp.sec + closest_it->stamp.nanosec * 1e-9 - image_time;
    }
    void ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg)
    {
        // 数据获取
        image_stamp_ = msg->header.stamp;
        current_time_ = image_stamp_.sec + image_stamp_.nanosec * 1e-9;
        const auto& armor_plates = msg->armor_plates;
        float yaw_abs = 0.0;
        float pitch_abs = 0.0;
        findClosestAngle(msg->header.stamp, yaw_abs, pitch_abs);
        tracker_.Update(armor_plates, current_time_, yaw_abs, pitch_abs);
        publish(msg);
    }
    void info()
    {
        float measurement_yaw = tracker_.getMeasuredYaw();
        float measurement_pitch = tracker_.getMeasuredPitch();
        float filter_yaw = tracker_.getYaw();
        float filter_pitch = tracker_.getPitch();
        RCLCPP_INFO(this->get_logger(), "测量数据: yaw = %.4f, pitch = %.4f||滤波数据: yaw = %.4f, pitch = %.4f",
            measurement_yaw, measurement_pitch, filter_yaw, filter_pitch);
    }
    void publishMarkerArray(const rclcpp::Time& now)
    {
        auto center = tracker_.getCenterPointWorld();
        visualization_msgs::msg::MarkerArray arr;
        // 旋转轴（绿色中心点 + 向上的绿色箭头)
        arr.markers.push_back(createSphereMarker(
            center, "world", now, 0,
            0.08f,
            0.0f, 1.0f, 0.0f, 1.0f
        ));
        Eigen::Vector3d axis_top = center + Eigen::Vector3d(0.0, 0.0, 0.5);
        arr.markers.push_back(createArrowMarker(
            center, axis_top, "world", now, 1,
            0.02f, 0.06f, 0.0f,
            0.0f, 1.0f, 0.0f, 1.0f
        ));
        // 中心速度箭头（黄色）
        Eigen::Vector3d velocity = tracker_.getCenterVelocity();
        constexpr double kVelocityScale = 0.5;
        Eigen::Vector3d arrow_end = center + velocity * kVelocityScale;
        arr.markers.push_back(createArrowMarker(
            center, arrow_end, "world", now, 2,
            0.02f, 0.06f, 0.0f,
            1.0f, 1.0f, 0.0f, 1.0f
        ));
        // 观测装甲板（红色）
        arr.markers.push_back(createBoxMarker(
            tracker_.getMeasuredPositionWorld(),
            tracker_.getMeasuredOrientationWorld(),
            "world", now, 3,
            1.0f, 0.0f, 0.0f, 1.0f
        ));
        // 滤波装甲板（绿色）
        arr.markers.push_back(createBoxMarker(
            tracker_.getFilterPositionWorld(),
            tracker_.getFilterOrientationWorld(),
            "world", now, 4,
            0.0f, 1.0f, 0.0f, 1.0f
        ));

        marker_array_pub_->publish(arr);
    }
    void publish(const ArmorPlates::SharedPtr armor_plates)
    {
        // AimCommand 和 TrackerData 仅在跟踪成功时发送
        if (!tracker_.isLost()) {
            AimCommand aim_command;
            aim_command.delta_pitch = tracker_.getPitch();
            aim_command.delta_yaw = tracker_.getYaw();
            aim_command_pub_->publish(aim_command);

            TrackerData tracker_data_msg;
            tracker_data_msg.header = armor_plates->header;
            tracker_data_msg.measurement_yaw = tracker_.getMeasuredYaw();
            tracker_data_msg.measurement_pitch = tracker_.getMeasuredPitch();
            tracker_data_msg.filter_yaw = tracker_.getYaw();
            tracker_data_msg.filter_pitch = tracker_.getPitch();
            if (tracker_data_pub_) {
                tracker_data_pub_->publish(tracker_data_msg);
            }
        }

        // 发布目标位姿 世界坐标系
        Eigen::Vector3d measured_pos_world = tracker_.getMeasuredPositionWorld();
        auto now = this->now();
        auto quatEigenToMsg = [](const Eigen::Quaterniond& q) {
            geometry_msgs::msg::Quaternion msg;
            msg.w = q.w();
            msg.x = q.x();
            msg.y = q.y();
            msg.z = q.z();
            return msg;
        };
        // TF发布到世界坐标系
        geometry_msgs::msg::TransformStamped filter_transform_stamped;
        filter_transform_stamped.header.stamp = now;
        filter_transform_stamped.header.frame_id = "world";
        filter_transform_stamped.child_frame_id = "id_1";
        filter_transform_stamped.transform.translation.x = measured_pos_world.x();
        filter_transform_stamped.transform.translation.y = measured_pos_world.y();
        filter_transform_stamped.transform.translation.z = measured_pos_world.z();
        filter_transform_stamped.transform.rotation = quatEigenToMsg(tracker_.getMeasuredOrientationWorld());
        tf_broadcaster_->sendTransform(filter_transform_stamped);
        // 发布可视化数据
        publishMarkerArray(now);
        ////////// DEBUG //////////
        if (debug_) {
            TrackerDebug debug_msg;
            debug_msg.header.stamp = image_stamp_;
            auto eigen2geometry = [](const Eigen::Vector3d& vec) {
                geometry_msgs::msg::Vector3 vec_msg;
                vec_msg.x = vec.x();
                vec_msg.y = vec.y();
                vec_msg.z = vec.z();
                return vec_msg;
            };
            if (!tracker_.isLost()) {
                Eigen::Vector3d measured_cam = tracker_.getMeasuredPositionCamera();
                debug_msg.target_point = eigen2geometry(measured_cam);
                Eigen::Vector3d filtered_cam = tracker_.getFilterPositionCamera();
                debug_msg.filtered_point = eigen2geometry(filtered_cam);
            } else {
                // 丢失时放在相机正前方（光轴上，投影到图像中心）
                geometry_msgs::msg::Vector3 center_point;
                center_point.x = 0.0;
                center_point.y = 0.0;
                center_point.z = 1.0;
                debug_msg.target_point = center_point;
                debug_msg.filtered_point = center_point;
            }

            tracker_debug_pub_->publish(debug_msg);
        }
    }

public:
    ArmorPlateTracker() : Node("armor_plate_tracker_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Armor Plate Tracker节点创建成功！");
        init();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateTracker>());
    rclcpp::shutdown();
    return 0;
}
