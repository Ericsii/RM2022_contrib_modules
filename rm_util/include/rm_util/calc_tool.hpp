#ifndef RM_UTIL__CALC_TOOL_HPP
#define RM_UTIL__CALC_TOOL_HPP

#include <vector>

#include "opencv2/opencv.hpp"

namespace rm_util
{
    template <typename T>
    inline T rad_to_deg(T radian)
    {
        return radian * 180 / CV_PI;
    }
    template <typename T>
    inline T deg_to_rad(T degree)
    {
        return degree / 180 * CV_PI;
    }

    // const definition of color
    const auto blue = cv::Scalar(255, 0, 0);
    const auto green = cv::Scalar(0, 255, 0);
    const auto red = cv::Scalar(0, 0, 255);

    // math function
    float calc_inclination_angle(cv::Point2f point1, cv::Point2f point2);
    float calc_inner_angle(cv::Point2f vertex_point, cv::Point2f point1, cv::Point2f point2);
    bool calc_circle_from_3points(
        cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f &point, float &r);

    // draw function
    void draw_rotated_rect(cv::Mat &img, cv::RotatedRect r, cv::Scalar color = green);
    void draw_4points(cv::Mat &img, const cv::Point2f *point2fs, cv::Scalar color = green);
    void draw_convex_hull(cv::Mat &img, std::vector<cv::Point2f> &points, cv::Scalar color = green);
} // namespace rm_util

#endif // RM_UTIL__CALC_TOOL_HPP