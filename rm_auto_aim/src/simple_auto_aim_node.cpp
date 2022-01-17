#include "rm_auto_aim/simple_auto_aim_node.hpp"

#include <string>
#include <sstream>

#include "sensor_msgs/msg/camera_info.hpp"
#include "rm_interfaces/msg/gimbal.hpp"
#include "rm_auto_aim/armor_detector_svm.hpp"

namespace rm_auto_aim
{
    SimpleAutoAimNode::SimpleAutoAimNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("simple_auto_aim", options);
        std::string camera_name = "camera";
        std::string imu_name = "imu";
        std::string robot_color = "red";
        bool auto_start = false;

        node_->declare_parameter("robot_color", robot_color);
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("imu_name", imu_name);
        node_->declare_parameter("auto_start", auto_start);
        node_->declare_parameter("xml_path", "");

        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("imu_name", imu_name);
        node_->get_parameter("robot_color", robot_color);
        node_->get_parameter("auto_start", auto_start);

        bool is_red = (robot_color == "red");

        using namespace std::placeholders;
        RCLCPP_INFO(node_->get_logger(), "Creating rcl pub&sub&client.");
        gimbal_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>(
            "/cmd_gimbal", 10);
        set_mode_srv_ = node_->create_service<rm_interfaces::srv::SetMode>(
            "auto_aim/set_mode", std::bind(&SimpleAutoAimNode::set_mode_cb, this, _1, _2));

        wrapper_client_ = std::make_shared<rm_cam::WrapperClient>(
            node_, camera_name, imu_name, std::bind(&SimpleAutoAimNode::process_fn, this, _1, _2, _3));
        RCLCPP_INFO(node_->get_logger(), "Create success.");

        sensor_msgs::msg::CameraInfo cam_info;
        if (!wrapper_client_->get_camera_info(cam_info))
        {
            RCLCPP_FATAL(
                node_->get_logger(),
                "Get camera info failed!");
            return;
        }

        std::ostringstream oss;
        oss << "k:";
        for (auto &x : cam_info.k)
        {
            oss << x << " ";
        }
        oss << ",d:";
        for (auto &x : cam_info.d)
        {
            oss << x << " ";
        }
        RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());

        // 初始化
        std::vector<double> camera_k(9, 0);
        std::copy_n(cam_info.k.begin(), 9, camera_k.begin());
        std::shared_ptr<rm_auto_aim::ArmorDetector> detector = std::make_shared<rm_auto_aim::ArmorDetectorSVM>(node_);
        auto_aim_algo_ = std::make_shared<rm_auto_aim::SimpleAutoAimAlgo>(node_, camera_k, cam_info.d, detector);
        auto_aim_algo_->set_target_color(!is_red);
        transform_tool_ = std::make_shared<rm_util::CoordinateTranslation>();
        measure_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_k, cam_info.d);
        RCLCPP_INFO(
            node_->get_logger(),
            "Init finished.");
    }

    void SimpleAutoAimNode::process_fn(std_msgs::msg::Header header, cv::Mat &img, geometry_msgs::msg::Pose pose)
    {
        // 计算时间戳
        double time_stamp_ms = header.stamp.sec * 1e3 + header.stamp.nanosec * 1e-6;

#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(
                node_->get_logger(),
                "Get message"
            );
#endif // RM_DEBUG_MODE
        // 姿态四元数
        curr_pose_ = Eigen::Quaterniond(pose.orientation.w,
                                        pose.orientation.x,
                                        pose.orientation.y,
                                        pose.orientation.z);

        int ret = auto_aim_algo_->process(time_stamp_ms, img, curr_pose_);
        if (!ret)
        {
            auto target = auto_aim_algo_->getTarget();
            float pitch, yaw;
            measure_tool_->calc_view_angle(target.armorDescriptor.centerPoint, pitch, yaw);
            pitch = rm_util::rad_to_deg(pitch);
            yaw = rm_util::rad_to_deg(yaw);

            auto euler_angles = rm_util::CoordinateTranslation::quat2euler(curr_pose_);
            float c_pitch, c_yaw;
            c_pitch = rm_util::rad_to_deg(euler_angles(1));
            c_yaw = rm_util::rad_to_deg(euler_angles(0));

#ifdef RM_DEBUG_MODE
            double q1, q2, q3, q0;
            // float c_pitch, c_yaw;
            q3 = pose.orientation.w;
            q0 = pose.orientation.x;
            q1 = pose.orientation.y;
            q2 = pose.orientation.z;
            c_yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)* 57.3;
            c_pitch = -asin(2.0f * (q1 * q3 - q0 * q2))* 57.3;
      
            RCLCPP_INFO(
                node_->get_logger(),
                "c_pitch: %f, c_yaw: %f",c_pitch,c_yaw
            );
            RCLCPP_INFO(
                node_->get_logger(),
                "offset_pitch: %f, offset_yaw: %f",pitch,yaw
            );
#endif    
            if (this->gimbal_ctrl_flag_)
            {
                rm_interfaces::msg::GimbalCmd gimbal_cmd;
                gimbal_cmd.id = gimbal_cmd_id++;
                gimbal_cmd.position.pitch = -pitch + c_pitch;
                gimbal_cmd.position.yaw = yaw + c_yaw;
                gimbal_cmd.velocity.pitch = 0;
                gimbal_cmd.velocity.yaw = 0;
                gimbal_cmd_pub_->publish(gimbal_cmd);
            }
            if (this->shoot_ctrl_flag_)
            {
            }
        }
        else{
#ifdef RM_DEBUG_MODE
            RCLCPP_INFO(
                node_->get_logger(),
                "No armors"
            );
#endif
        }
    }

    bool SimpleAutoAimNode::set_mode_cb(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
    {
        // 0x00,休眠模式，0x01:自动射击模式，0x02：自动瞄准模式（不发子弹）,0x03,测试模式,不控制.
        // 0x10,设置目标为红色，0x11,设置目标为蓝色
        response->success = true;
        switch (request->mode)
        {
        case 0x00:
            wrapper_client_->stop();
            break;
        case 0x01:
            gimbal_ctrl_flag_ = true;
            shoot_ctrl_flag_ = true;
            wrapper_client_->start();
            break;
        case 0x02:
            gimbal_ctrl_flag_ = true;
            shoot_ctrl_flag_ = false;
            wrapper_client_->start();
            break;
        case 0x03:
            gimbal_ctrl_flag_ = false;
            shoot_ctrl_flag_ = false;
            wrapper_client_->start();
            break;
        case 0x10:
            auto_aim_algo_->set_target_color(true);
            RCLCPP_INFO(node_->get_logger(), "set target color red");
            break;
        case 0x11:
            auto_aim_algo_->set_target_color(false);
            RCLCPP_INFO(node_->get_logger(), "set target color blue");
            break;
        default:
            response->success = false;
        }
        return true;
    }
} // namespace rm_auto_aim


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::SimpleAutoAimNode)