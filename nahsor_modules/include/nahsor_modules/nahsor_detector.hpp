/*
 * @Author: your name
 * @Date: 2021-11-17 13:30:34
 * @LastEditTime: 2021-11-30 21:53:22
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scu_rm_ros/src/nahsor_modules/include/nahsor_modules/nahsor_detector.hpp
 */
#ifndef NAHSOR_MODULES_NAHSOR_DETECTOR_H
#define NAHSOR_MODULES_NAHSOR_DETECTOR_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "nahsor_modules/rune.hpp"
#include "rm_util/calc_tool.hpp"
#include <cmath>
namespace nahsor
{
    class Circle
    {
    public:
        cv::Point2f R_center;
        double radius;
    };
    class NahsorDetector
    {
    public:
        /**
         * @brief Construct a new NahsorAlgo Detector object
         * 
         * @param node rclcpp node节点
         */
        NahsorDetector(
            rclcpp::Node::SharedPtr node);

        /**
         * @brief 图像预处理
         * 
         * @param src 原图像
         * @param dst 目标图像
         * @return int 是否成功执行 1->失败，0->成功
         */
        int preProcess(cv::Mat &src, cv::Mat &dst);
        /**
         * @brief 检测装甲板位置
         * 
         * @param img 图像
         */
        void detect(cv::Mat &img);
        /**
         * @brief Get the Result object
         * 
         * @param rune 
         * @return cv::Point3f 
         */
        cv::Point3f getResult(Rune rune);
        /**
         * @brief 筛选出能量机关的装甲板
         * 
         * @param rect 经过初步筛选的轮廓 
         * @param armor 目标装甲版
         * @return int 是否成功执行 1->失败，0->成功
         */
        int getArmorDescriptor(cv::RotatedRect rect, Rune &armor);
        /**
         * @brief 获取圆心坐标
         * 
         * @param rect_parent 目标装甲板的父轮廓
         */
        void getR(Rune &rect_armor, cv::RotatedRect &rect_parent);
        /**
         * @brief 计算两点距离
         * 
         * @param point1 第一个点
         * @param point2 第二个点
         * @return double 距离
         */
        double calc_dis(cv::Point2f &point1, cv::Point2f &point2);
        /**
         * @brief 计算两点角度
         * 
         * @param point1 第一个点
         * @param point2 第二个点
         * @return double 角度
         */
        double calc_angle(cv::Point2f &point1, cv::Point2f &point2);
        /**
         * @brief Get the armors object
         * 
         * @return std::vector<Rune> 
         */
        std::vector<Rune> get_armors();
        /**
         * @brief Get the circle object
         * 
         * @return Circle 
         */
        Circle get_circle();
    private:
        rclcpp::Node::SharedPtr node_;
        std::string color_;
        bool debug_;
        int mode_;
        bool USE_HSV_;
        std::map<std::string, std::vector<int>> config_;
        std::vector<Rune> armors_;
        std::vector<cv::RotatedRect> R_edge_;
        //std::vector<double> armor_width_thre_;
        //std::vector<double> armor_height_thre_;
        //std::vector<double> armor_area_thre_;
        //std::vector<double> armor_ratio_thre_;
        //double max_r_angle_;
        Circle circle;
    };

}
#endif