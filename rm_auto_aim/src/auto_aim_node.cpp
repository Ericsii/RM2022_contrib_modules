#include "rm_auto_aim/auto_aim_node.hpp"

namespace rm_auto_aim
{
    AutoAimNode::AutoAimNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("auto_aim_node", options);
        std::string camera_name;
        std::string svm_path;
        bool armor_is_red;

        node_->declare_parameter("armor_is_red", armor_is_red);
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("svm_path", svm_path);
        node_->get_parameter("armor_is_red", armor_is_red);
        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("svm_path", svm_path);

        cam_client_ = std::make_shared<rm_cam::CamClient>(
            node_, 
            camera_name, 
            std::bind(
                &AutoAimNode::ProcessImage, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2), 
            true);

        RCLCPP_INFO(
            node_->get_logger(),
            "Target color: %s", armor_is_red ? "red" : "blue"
        );
        RCLCPP_INFO(
            node_->get_logger(),
            "SVM load path: %s", svm_path.c_str()
        );
        armor_detector_ = std::make_shared<ArmorDetectorSVM>(armor_is_red, svm_path);

        cam_client_->start();
    }

    void AutoAimNode::ProcessImage(cv::Mat& img, double time_stamp_ms)
    {
        (void)time_stamp_ms;
        ArmorData target = armor_detector_->GetTargetArmor(img);
        (void)target;
    }
}