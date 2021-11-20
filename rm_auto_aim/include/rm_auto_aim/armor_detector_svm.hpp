#ifndef RM_AUTO_AIM__ARMOR_DETECTOR_SVM_HPP_
#define RM_AUTO_AIM__ARMOR_DETECTOR_SVM_HPP_


#include <iostream>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#define SCORE_BEGAIN 40      //分数许可最小值
#define LOST_MAX 8          //掉帧缓冲
#define MIN_DISTANCE 120 //两次相同目标距离

namespace rm_auto_aim
{
    //射击模式
    enum Pattern
    {
        first_find,        //首次识别
        shoot,            //连续射击
        stop,             //非连续
        buffering          //缓冲
    };
    struct Led
    {
        RotatedRect box;              //拟合椭圆
        Point2f led_top = Point2f(0,0);          //灯条上点坐标
        Point2f led_bottom = Point2f(0,0);    //灯条下点坐标
    };
    struct ArmorData
    {
        Pattern statu = stop;
        bool is_small = true;
        int armor_num = 0;
        Led leds_[2];
        Point2f point[4];
    };


    class ArmorDetectorSVM
    {
        private:
            //定义识别装甲颜色
            bool armor_is_red_ = true;
            //记录掉帧次数
            int lost_number = LOST_MAX;          
            //装甲长宽比范围
            float small_max_ratio = 2.65;
            float small_min_ratio = 0.9;
            float big_max_ratio = 6.0;
            float big_min_ratio = 2.7;
            //装甲符号
            int shoot_armor_number_;
            vector<Led> leds_;
            vector<ArmorData> armor_datas_;
            ArmorData armor_old_;
            Ptr<ml::SVM> svm_;

            void PreDeal(Mat& src, Mat &gray);
            bool GetLeds(Mat& gray);
            bool GetArmor();
            bool GetBestArmor(Mat& src,ArmorData& best_armor);
            int GetNumber(Mat& src,Point2f center,float height,float angle);

        public:
            /**
             * @brief Construct a new Armor Detector S V M object
             * 
             * @param is_red : Target armor color;
             */
            ArmorDetectorSVM(bool armor_is_red, string xml_path)
            {
                armor_is_red_ = armor_is_red;
                svm_ = ml::StatModel::load<ml::SVM>(xml_path);
            }

            /**
             * @brief Get the Target Armor object
             * 
             * @param src : Source image.
             * @return ArmorData : Target armor data's struct type.
             */
            ArmorData GetTargetArmor(Mat &src);
    };
}

#endif  // ARMOR_DETECTOR_SVM_HPP_