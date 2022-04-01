#ifndef SENTRY_NODE_HPP_
#define SENTRY_NODE_HPP_
#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_interfaces/msg/gimbal.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_cam/cam_client.hpp"
#include "rm_util/measure_tool.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rm_cam/wrapper_client.hpp"
#include "rm_util/coordinate_tool.hpp"
#include "sentry_algo.hpp"

namespace rm_sentry
{
    class SentryNode
    {
    public:
        explicit SentryNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

    private:
        void process_fn(std_msgs::msg::Header header,
                        cv::Mat &img,
                        geometry_msgs::msg::Quaternion q);

        bool set_mode_cb(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                         std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rm_cam::CamClient> cam_client_;
        std::shared_ptr<rm_cam::WrapperClient> wrapper_client_;
        std::shared_ptr<rm_util::CoordinateTranslation> transform_tool_; // 坐标变换工具
        std::shared_ptr<rm_util::MonoMeasureTool> measure_tool_;         // 单目测量类
        std::shared_ptr<SentryAlgo> sentry_algo_;

        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
        rclcpp::Subscription<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
        Eigen::Quaterniond curr_pose_;
        bool gimbal_ctrl_flag_{true};
        bool shoot_ctrl_flag_{true};
        u_int8_t gimbal_cmd_id{0};
    };
}
#endif