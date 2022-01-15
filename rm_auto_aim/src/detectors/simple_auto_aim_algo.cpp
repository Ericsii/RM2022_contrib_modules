#include "rm_auto_aim/simple_auto_aim_algo.hpp"

#include <iostream>
#include <algorithm>
#include <Eigen/Core>

#include "rm_auto_aim/armor_detector_svm.hpp"

namespace rm_auto_aim
{
    SimpleAutoAimAlgo::SimpleAutoAimAlgo(rclcpp::Node::SharedPtr node,
                             std::vector<double> camera_intrinsic,
                             std::vector<double> camera_distortion,
                             std::shared_ptr<ArmorDetector> armor_detector) : node_(node), armor_detector_(armor_detector)
    {
        mono_loacation_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_intrinsic, camera_distortion);

        float realWidth, realHeight, half_x, half_y;

        // 小装甲
        realHeight = 6.3;
        realWidth = 12.8;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mSmallArmorPoints.emplace_back(-half_x, half_y, 0.);
        mSmallArmorPoints.emplace_back(-half_x, -half_y, 0);
        mSmallArmorPoints.emplace_back(half_x, -half_y, 0);
        mSmallArmorPoints.emplace_back(half_x, half_y, 0);

        // 大装甲
        realWidth = 22.2;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mBigArmorPoints.emplace_back(-half_x, half_y, 0.);
        mBigArmorPoints.emplace_back(-half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, half_y, 0);
        
        mono_loacation_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_intrinsic, camera_distortion);
    }

    int SimpleAutoAimAlgo::process(double time_stamp, cv::Mat &src, Eigen::Quaterniond pose)
    {
        // 仅供测试使用，无弹道补偿和滤波跟踪
        using std::vector;
        using std::sort;

        (void) time_stamp;
        (void) pose;
        // 陀螺仪四元数、坐标系转换四元数
        // Eigen::Quaterniond imu_q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        // Eigen::Matrix3d rotation(imu_q.matrix());

        auto img_center_point = cv::Point2f(src.cols / 2, src.rows / 2);
        // step1.图像识别
        int ret;
        ret = armor_detector_->process(src);
        if (ret != 0)
        {
            return 1; // 无目标
        }
        auto armor_descriptors = armor_detector_->getArmorVector();

#ifdef RM_DEBUG_MODE
    RCLCPP_INFO(
        node_->get_logger(),
        "detected armor size: %d", armor_descriptors.size());
#endif
        
        // 按照图像中心远近排序
        sort(armor_descriptors.begin(), armor_descriptors.end(), [img_center_point](const ArmorDescriptor &a, const ArmorDescriptor &b){
            auto dis_a = cv::norm(img_center_point - a.centerPoint);
            auto dis_b = cv::norm(img_center_point - b.centerPoint);
            return dis_a < dis_b;
        });

        mTarget.armorDescriptor = armor_descriptors.front();
        return 0;

        // // step2.位置解算
        // vector<ArmorTarget> armor_target_vec;
        // for (size_t i = 0; i < armor_descriptors.size(); ++i)
        // {
        //     ArmorTarget armor_target;
        //     armor_target.armorDescriptor = armor_descriptors[i];
        //     auto label = armor_descriptors[i].label;
        //     vector<cv::Point2f> points;
        //     points.push_back(armor_descriptors[i].points[0]);
        //     points.push_back(armor_descriptors[i].points[1]);
        //     points.push_back(armor_descriptors[i].points[2]);
        //     points.push_back(armor_descriptors[i].points[3]);

        //     if (label == 0 || label == 1 || label == 8)
        //     {
        //         // 大装甲板匹配
        //         mono_loacation_tool_->solve_pnp(points, mBigArmorPoints,
        //                                         armor_target.postion, armor_target.rotation);
        //         armor_target.isBigArmor = true;
        //     }
        //     else
        //     {
        //         // 小装甲板匹配
        //         mono_loacation_tool_->solve_pnp(points, mSmallArmorPoints,
        //                                         armor_target.postion, armor_target.rotation);
        //     }
        // }
    }

    ArmorTarget SimpleAutoAimAlgo::getTarget()
    {
        return this->mTarget;
    }

    void SimpleAutoAimAlgo::set_target_color(bool is_red)
    {
        armor_detector_->set_target_color(is_red);
    }

    void SimpleAutoAimAlgo::setTrack(bool is_track)
    {
        mIsTrack = is_track;
    }
}