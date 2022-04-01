#ifndef SENTRY_ALGO_HPP
#define SENTRY_ALGO_HPP
#include <opencv2/opencv.hpp>
#include "rm_util/rm_util.hpp"
#include "rm_sentry/sentry_detector.hpp"
#include "rm_util/measure_tool.hpp"
#include "rm_filters/ekf_filter.hpp"
#include "rm_trajectory/trajectory_pitch.hpp"
#include <opencv2/core/eigen.hpp>

namespace rm_sentry
{
    class SentryAlgo
    {
    public:
        SentryAlgo(rclcpp::Node::SharedPtr node,
                   std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool,
                   bool target_color_red, std::string &xml_path, std::string &bin_path, double threhold);
        int process(double time_stamp_ms, cv::Mat &src, Eigen::Quaterniond pose);

        bool is_same_armor(Eigen::Vector3d old_position3d, Eigen::Vector3d now_position3d);

        rm_auto_aim::ArmorTarget getTarget();
        float mTarget_pitch = 0;
        float mTarget_yaw = 0;
        double mTarget_distance = 0;
        double mTarget_height = 0;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool_;
        SentryDetector sentry_detector_;
        bool last_shoot;
        Bbox last_box;
        bool selected;
        Eigen::Quaterniond cam2imu_static_;
        rm_auto_aim::ArmorDescriptor target_armor;
        std::vector<cv::Point3f> mSmallArmorPoints; // 小装甲三维点
        std::vector<cv::Point3f> mBigArmorPoints;   // 大装甲三维点
        rm_auto_aim::ArmorTarget mTarget;           // 最终目标
        rm_filters::Filters *ekf_filter;
        Eigen::Matrix3d filter_position3d_world;
        Eigen::MatrixXd U;
        Eigen::Matrix3d F;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        Eigen::Vector3d p_t;
        Eigen::MatrixXd z_k;
        Eigen::Vector3d last_position3d;
        rm_auto_aim::ArmorTarget armor_target;
        rm_auto_aim::ArmorTarget last_target;
        std::vector<cv::Point2f> points;
        double last_time = 0;
    };
}
#endif