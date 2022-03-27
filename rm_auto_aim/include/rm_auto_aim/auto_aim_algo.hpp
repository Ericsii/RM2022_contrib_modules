#ifndef RM_AUTO_AIM__AUTO_AIM_ALGO_HPP
#define RM_AUTO_AIM__AUTO_AIM_ALGO_HPP

#include <string>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/eigen.hpp>

#include "std_msgs/msg/header.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "rm_auto_aim/armor_detector_interface.hpp"
#include "rm_trajectory/trajectory_pitch.hpp"
#include "rm_filters/ekf_filter.hpp"
#include "rm_util/rm_util.hpp"

namespace rm_auto_aim
{
    class AutoAimAlgo
    {
    public:
        AutoAimAlgo(rclcpp::Node::SharedPtr node,
                          std::vector<double> camera_intrinsic,
                          std::vector<double> camera_distortion,
                          std::shared_ptr<ArmorDetector> armor_detector);

        void set_target_color(bool is_red);
        int process(double time_stamp_ms, cv::Mat &src, Eigen::Quaterniond pose, int aim_mode);
        ArmorTarget getTarget();
        void setTrack(bool is_track);
        bool is_same_armor(Eigen::Vector3d old_position3d, Eigen::Vector3d now_position3d, double distance_threshold);

        float mTarget_pitch = 0;
        float mTarget_yaw = 0;
        double mTarget_distance = 0;
        double mTarget_height = 0;

    private:
        rclcpp::Node::SharedPtr node_;                                  // rclcpp 节点
        std::shared_ptr<ArmorDetector> armor_detector_;                 // 装甲板检测对象
        std::shared_ptr<rm_util::MonoMeasureTool> mono_loacation_tool_; // 单目测量工具

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cam_point_pub_;

        std::vector<cv::Point3f> mSmallArmorPoints; // 小装甲三维点
        std::vector<cv::Point3f> mBigArmorPoints;   // 大装甲三维点
        ArmorTarget mTarget;                        // 最终目标
        bool mIsTrack;

        Eigen::Quaterniond cam2imu_static_;
        Eigen::Quaterniond imu2cam_static_;
        double last_yaw = 0;
        int last_label = -1;

        Eigen::Vector3d last_position3d_world;
        rm_filters::Filters *ekf_filter;
        Eigen::Matrix3d f_position3d_world;
        Eigen::Matrix3d filter_position3d_world;
        Eigen::MatrixXd U;
        Eigen::Matrix3d F;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        Eigen::Vector3d p_t;

        double last_time = 0;
        int id = 0;
        int all = 0;
        double time_bet = 10;
        // double aim_round_time = ;
        double aim_range = 5;
        double last_armor_area = 0;
        double last_armor_area_rot = 0;
        double auto_aim_time[10]; // [1]begin，[2]max，[3]end，[0]round-T
        std::string end_pre = "normal"; // normal right left
    };
} // namespace rm_auto_aim

#endif // RM_AUTO_AIM__AUTO_AIM_ALGO_HPP