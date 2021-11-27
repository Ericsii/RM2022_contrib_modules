#ifdef DEBUG_MODE
#include <ctime>
#endif
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

#ifdef DEBUG_MODE
        RCLCPP_INFO(
            node_->get_logger(),
            "Target color: %s", armor_is_red ? "red" : "blue"
        );
        RCLCPP_INFO(
            node_->get_logger(),
            "SVM load path: %s", svm_path.c_str()
        );
#endif

        cam_client_ = std::make_shared<rm_cam::CamClient>(
            node_, 
            camera_name, 
            std::bind(
                &AutoAimNode::ProcessImage, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2), 
            true);

        armor_detector_ = std::make_shared<ArmorDetectorSVM>(armor_is_red, svm_path);
        
        cam_client_->start();
    }

    void AutoAimNode::ProcessImage(cv::Mat& img, double time_stamp_ms)
    {
        (void)time_stamp_ms;

#ifdef DEBUG_MODE
        clock_t start,end;
        start = clock();
#endif

        std::vector<ArmorData> target = armor_detector_->GetArmors(img);

#ifdef DEBUG_MODE
        end = clock();
#endif

#ifdef DEBUG_MODE
        //绘制矩形,即待击打装甲板
	    double endtime=(double)(end-start)/CLOCKS_PER_SEC;
        if(time_sum_ > 1)
        {
            std::cout << "FPS: " << fps_ << std::endl;
            time_sum_ = 0;
            fps_ = 0;
        }
        else
        {
            fps_++;
            time_sum_ += endtime;
        }
        for(size_t i = 0; i < target.size(); i++)
        {
            std::cout << "Time cost: " << endtime << std::endl;
            cv::line(img, target[i].point[0], target[i].point[1], cv::Scalar(0,255,140), 1);
            cv::line(img, target[i].point[1], target[i].point[2], cv::Scalar(0,255,140), 1);
            cv::line(img, target[i].point[2], target[i].point[3], cv::Scalar(0,255,140), 1);
            cv::line(img, target[i].point[3], target[i].point[0], cv::Scalar(0,255,140), 1);
            cv::putText(img, std::to_string(target[i].armor_num), target[i].point[0], cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0,255,140));
        }
        cv::imshow("result",img);
        cv::waitKey(1);
#endif
    }
}