#ifndef RM_AUTO_AIM__AUTO_AIM_ALGO_HPP
#define RM_AUTO_AIM__AUTO_AIM_ALGO_HPP

#include <string>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rm_auto_aim/armor_detector_interface.hpp"
#include "rm_trajectory/trajectory_pitch.hpp"
#include "rm_filters/ekf_filter.hpp"
#include "rm_util/rm_util.hpp"

namespace rm_auto_aim
{
    class SimpleAutoAimAlgo
    {
    public:
        SimpleAutoAimAlgo(rclcpp::Node::SharedPtr node,
                          std::vector<double> camera_intrinsic,
                          std::vector<double> camera_distortion,
                          std::shared_ptr<ArmorDetector> armor_detector);

        void set_target_color(bool is_red);

        int process(double time_stamp_ms, cv::Mat &src, Eigen::Quaterniond pose);

        ArmorTarget getTarget();

        void setTrack(bool is_track);
        void is_same_armor(const Eigen::Vector3d old_position3d, const Eigen::Vector3d now_position3d, const double distance_threshold);

    private:
        rclcpp::Node::SharedPtr node_;                                  // rclcpp 节点
        std::shared_ptr<ArmorDetector> armor_detector_;                 // 装甲板检测对象
        std::shared_ptr<rm_util::MonoMeasureTool> mono_loacation_tool_; // 单目测量工具

        std::vector<cv::Point3f> mSmallArmorPoints; // 小装甲三维点
        std::vector<cv::Point3f> mBigArmorPoints;   // 大装甲三维点
        ArmorTarget mTarget;                        // 最终目标
        bool mIsTrack;
        double last_yaw = 0;
        rm_filters::Filters *ekf_filter;
        Eigen::MatrixXd z_k;
    };
} // namespace rm_auto_aim

#endif // RM_AUTO_AIM__AUTO_AIM_ALGO_HPP