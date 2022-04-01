/*
 * @Author: your name
 * @Date: 2021-11-17 11:43:44
 * @LastEditTime: 2021-12-01 22:37:13
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scu_rm_ros/src/nahsor_modules/include/nahsor_node.hpp
 */
#ifndef NAHSOR_NODE_HPP_
#define NAHSOR_NODE_HPP_

#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_interfaces/msg/gimbal.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "nahsor_modules/nahsor_prediction.hpp"
#include "rm_cam/cam_client.hpp"
#include "nahsor_modules/nahsor.hpp"
#include "rm_util/measure_tool.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rm_cam/wrapper_client.hpp"
#include "rm_util/coordinate_tool.hpp"
#include "rm_trajectory/trajectory_pitch.hpp"
namespace nahsor
{
    enum class TaskMode
    {
        idle,
        small,
        large,
        test
    };

    class NahsorNode
    {
    public:
        explicit NahsorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

        // void process_image(cv::Mat &img, double time_stamp);

    private:
        /**
         * @brief 主流程函数，获取装甲板预测位置
         * 
         * @param img 相机获取的图像
         * @param time_stamp 时间戳
         */
        // void process_image(cv::Mat &img, double time_stamp);
        void process_image(std_msgs::msg::Header header, cv::Mat &img, geometry_msgs::msg::Pose pose);
        /**
         * @brief 更改目标模式的回调函数
         * 
         * @param request 
         * @param response 
         * @return true 
         * @return false 
         */
        bool setModeCallBack(
            const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
            std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
        void gimbalStateCallback(const rm_interfaces::msg::Gimbal::SharedPtr msg);
        void taskImageProcess(cv::Mat &img, double img_stamp);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rm_cam::CamClient> cam_client_;
        std::shared_ptr<rm_cam::WrapperClient> wrapper_client_;

        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_ctrl_pub_;
        rclcpp::Subscription<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

        std::shared_ptr<NahsorAlgo> nahsor_algo;
        std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool;
        TaskMode current_mode_{TaskMode::idle};
        bool is_need_clear_{false};
        bool is_need_reshoot_{false};
        int imgCount{0}; //是否优化拟合完成
        Eigen::Quaterniond curr_pose_;
        int count{0}; //判断优化边的数量
        u_int8_t gimbal_cmd_id{0};
    };
}

#endif