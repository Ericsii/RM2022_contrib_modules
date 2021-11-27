#ifndef RM_AUTO_AIM__AUTO_AIM_NODE_HPP_
#define RM_AUTO_AIM__AUTO_AIM_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "rm_cam/cam_client.hpp"
#include "rm_auto_aim/armor_detector_svm.hpp"

namespace rm_auto_aim
{
    class AutoAimNode
    {   
        public:
#ifdef DEBUG_MODE
            double time_sum_ = 0;
            int fps_ = 0;
#endif
            AutoAimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
            {
                return node_->get_node_base_interface();
            }
        private:
            void ProcessImage(cv::Mat& img, double time_stamp_ms);

        private:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<rm_auto_aim::ArmorDetectorSVM> armor_detector_;
            std::shared_ptr<rm_cam::CamClient> cam_client_;
    };
}

#endif