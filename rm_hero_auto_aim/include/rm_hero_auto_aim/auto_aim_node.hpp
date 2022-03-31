#ifndef RM_AUTO_AIM__AUTO_AIM_NODE_HPP
#define RM_AUTO_AIM__AUTO_AIM_NODE_HPP

#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rm_util/rm_util.hpp"
#include "rm_hero_auto_aim/armor_detector_interface.hpp"
#include "rm_hero_auto_aim/auto_aim_algo.hpp"

#include "rm_cam/wrapper_client.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/srv/set_mode.hpp"

namespace rm_hero_auto_aim
{
    class AutoAimNode
    {
    public:
        explicit AutoAimNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

    private:
        void process_fn(
            std_msgs::msg::Header header,
            cv::Mat &img,
            geometry_msgs::msg::Quaternion pose);
        bool set_mode_cb(
            const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
            std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rm_cam::WrapperClient> wrapper_client_;          // 数据接收客户端
        std::shared_ptr<rm_hero_auto_aim::AutoAimAlgo> auto_aim_algo_;  // 自瞄算法
        std::shared_ptr<rm_util::CoordinateTranslation> transform_tool_; // 坐标变换工具
        std::shared_ptr<rm_util::MonoMeasureTool> measure_tool_;         // 单目测量类

        // ros pub
        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
        // ros srv
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

        bool gimbal_ctrl_flag_{true};
        bool shoot_ctrl_flag_{true};
        bool guard_ctrl_flag_{false};
        Eigen::Quaterniond curr_pose_;
        u_int8_t gimbal_cmd_id{0};
    };
}

#endif // RM_AUTO_AIM__AUTO_AIM_NODE_HPP