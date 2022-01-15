#ifndef RM_UTIL__MEASURE_TOOL_HPP
#define RM_UTIL__MEASURE_TOOL_HPP

#include <vector>

#include "opencv2/opencv.hpp"

namespace rm_util
{

    // 单目测距类
    class MonoMeasureTool
    {
    public:
        /**
         * @brief Construct a new Mono Measure Tool object
         *
         * @param camera_intrinsic 相机内参3*3=9
         * @param camera_distortion 相机畸变参数1*5
         */
        MonoMeasureTool(std::vector<double> camera_intrinsic, std::vector<double> camera_distortion);

        /**
         * @brief Set the camera info object
         *
         * @param camera_intrinsic 相机内参3*3=9
         * @param camera_distortion 相机畸变参数1*5
         * @return true 设置成功
         * @return false 设置失败
         */
        bool set_camera_info(std::vector<double> camera_intrinsic, std::vector<double> camera_distortion);

        /**
         * @brief 使用pnp求解深度和旋转信息
         *
         * @param points2d 图像坐标上点集
         * @param points3d 3d坐标上对应点集
         * @param position 返回的3维坐标系中位置
         * @param rotation 返回相对旋转向量
         * @return true
         * @return false
         */
        bool solve_pnp(
            std::vector<cv::Point2f> &points2d,
            std::vector<cv::Point3f> &points3d,
            cv::Point3f &position,
            cv::Mat &rotation);

        /**
         * @brief 相机逆投影
         * 已知图像坐标和深度信息，计算三维坐标(相机坐标系下)
         * @param p 图像像素坐标
         * @param distance 深度
         * @return cv::Point3f 三维坐标
         */
        cv::Point3f unproject(cv::Point2f p, double distance);

        /**
         * @brief 获取image任意点的视角，pitch，yaw（相对相机坐标系）。
         *
         * @param p 图像像素坐标
         * @param pitch 视角pitch
         * @param yaw 视角yaw
         */
        void calc_view_angle(cv::Point2f p, float &pitch, float &yaw);

    private:
        // 相机参数
        cv::Mat camera_intrinsic_;  // 相机内参3*3
        cv::Mat camera_distortion_; // 相机畸变参数1*5
    };

} // namespace rm_util

#endif // RM_UTIL__MEASURE_TOOL_HPP