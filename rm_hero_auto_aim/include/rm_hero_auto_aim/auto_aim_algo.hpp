#ifndef RM_AUTO_AIM__AUTO_AIM_ALGO_HPP
#define RM_AUTO_AIM__AUTO_AIM_ALGO_HPP

#include <string>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rm_hero_auto_aim/armor_detector_interface.hpp"
#include "rm_util/rm_util.hpp"
#include "rm_trajectory/trajectory_pitch.hpp"
#include "rm_filters/ekf_filter.hpp"

namespace rm_hero_auto_aim
{
    class AutoAimAlgo
    {
    public:
        /**
         * @brief 自瞄算法节点构造函数
         * 
         * @param node ： rclcpp 节点
         * @param camera_intrinsic ： 相机内参
         * @param camera_distortion ： 相机畸变参数
         * @param armor_detector ： 装甲板检测类，见armor_detector_svm.hpp
         * @param guard_mode ： 击打哨兵的处理逻辑
         */
        AutoAimAlgo(rclcpp::Node::SharedPtr node,
                          std::vector<double> camera_intrinsic,
                          std::vector<double> camera_distortion,
                          std::shared_ptr<ArmorDetector> armor_detector);

        /**
         * @brief 结合下位机返回的数据对图像进行处理
         * 
         * @param time_stamp_ms ： 当前图像时间戳
         * @param src ： 待处理图像
         * @param pose ： 姿态四元数
         * @return int ： 0）检测成功 1）未检测到目标
         */
        int process(double time_stamp_ms, cv::Mat &src, Eigen::Quaterniond pose);

        /**
         * @brief 判断旋转状态
         * 
         * @return int ： 0）非旋转状态 1）可追踪旋转状态 3）高速旋转状态
         */
        int judgeRotation();

        /**
         * @brief 攻击哨兵
         * 
         */
        void attackGuard();

        /**
         * @brief 设置目标颜色
         * 
         * @param is_red ： 目标是否为红色
         */
        void setTargetColor(bool is_red);

        /**
         * @brief 获得目标装甲板
         * 
         * @return ArmorTarget ： 描述目标装甲板的信息结构体，见armor_detector_interface.hpp
         */
        ArmorTarget getTarget();

        /**
         * @brief 设置是否追踪目标
         * 
         * @param is_track ： 是否追踪
         */
        void setTrack(bool is_track);

        /**
         * @brief 判断是否为同一块装甲板
         * 
         * @param old_position3d : 旧装甲板3D位置
         * @param now_position3d : 新装甲板3D位置
         * @param distance_threshold : 距离阈值
         */
        void isSameArmor(const Eigen::Vector3d old_position3d, const Eigen::Vector3d now_position3d, const double distance_threshold);


    private:
        rclcpp::Node::SharedPtr node_;                                  // rclcpp 节点
        std::shared_ptr<ArmorDetector> armor_detector_;                 // 装甲板检测对象
        std::shared_ptr<rm_util::MonoMeasureTool> mono_loacation_tool_; // 单目测量工具

        std::vector<cv::Point3f> m_small_armor_points_; // 小装甲三维点
        std::vector<cv::Point3f> m_big_armor_points_;   // 大装甲三维点
        ArmorTarget m_target_;                          // 最终目标
        
        std::vector<cv::Point2f> armors_centers_;        // 靠近屏幕中心装甲板的中心点
        int effective_armors_quantity_;                 // 判断旋转状态的最少有效装甲板数
        int fast_rotate_jump_count_;                    // 快速旋转的跳跃阈值
        bool m_is_track_;                               // 是否追踪
        bool guard_mode_;                               // 是否为击打哨兵模式，是则由哨兵逻辑接管处理算法
 
        double last_yaw = 0;                            //记录上次的YAW值
        rm_filters::Filters *ekf_filter;                //EKF滤波
        // Eigen::MatrixXd z_k;          
    }; // namespace rm_hero_auto_aim
}
#endif // RM_AUTO_AIM__AUTO_AIM_ALGO_HPP