/*
 * @Author: your name
 * @Date: 2021-11-17 14:47:46
 * @LastEditTime: 2021-11-30 15:09:42
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scu_rm_ros/src/nahsor_modules/include/nahsor_modules/rune.hpp
 */
#ifndef NAHSOR_MODULES_RUNE_H
#define NAHSOR_MODULES_RUNE_H
#include "opencv2/opencv.hpp"
class Rune
{
public:
    int id;
    cv::RotatedRect rect;
    cv::Point2f points[4];
    cv::Point2f center;
    float armor_hight;
    float armor_width;
    float armor_area;
    float armor_ratio_wh; //宽高比
    float angle;
};
#endif