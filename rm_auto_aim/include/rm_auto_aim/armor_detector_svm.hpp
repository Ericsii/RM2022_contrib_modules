#ifndef RM_AUTO_AIM__ARMOR_DETECTOR_SVM_HPP_
#define RM_AUTO_AIM__ARMOR_DETECTOR_SVM_HPP_

#include <iostream>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

namespace rm_auto_aim
{
    struct Led
    {
        cv::RotatedRect box;                        //拟合椭圆
        cv::Point2f led_top = cv::Point2f(0, 0);    //灯条上点坐标
        cv::Point2f led_bottom = cv::Point2f(0, 0); //灯条下点坐标
    };
    struct ArmorData
    {
        bool is_small = true;
        int armor_num = 0;
        Led leds_[2];
        cv::Point2f point[4];
    };

    class ArmorDetectorSVM
    {
    private:
        //定义识别装甲颜色
        bool armor_is_red_ = true;
        //装甲长宽比范围
        float small_max_ratio = 2.65;
        float small_min_ratio = 0.9;
        float big_max_ratio = 6.0;
        float big_min_ratio = 2.7;
        //装甲符号
        int shoot_armor_number_;
        std::vector<Led> leds_;
        std::vector<ArmorData> armor_datas_;
        ArmorData armor_old_;
        cv::Ptr<cv::ml::SVM> svm_;

        void PreDeal(cv::Mat &src, cv::Mat &gray);
        bool GetLeds(cv::Mat &gray);
        bool GetArmor(cv::Mat &src);
        int GetNumber(cv::Mat &src, cv::Point2f center, float height, float angle);

    public:
        ArmorDetectorSVM(bool armor_is_red, std::string xml_path)
        {
            armor_is_red_ = armor_is_red;
            svm_ = cv::ml::StatModel::load<cv::ml::SVM>(xml_path);
        }

        std::vector<ArmorData> GetArmors(cv::Mat &src);
    };
}

#endif // ARMOR_DETECTOR_SVM_HPP_