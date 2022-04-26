#ifndef RM_UTIL__DEBUG_IMAGE_HPP_
#define RM_UTIL__DEBUG_IMAGE_HPP_

#include <exception>
#include <vector>
#include "opencv2/opencv.hpp"

namespace rm_util
{
    namespace debug_image
    {
        inline cv::Mat draw_scatter(const std::vector<float> &x, const std::vector<float> &y, const cv::Scalar &color, int size = 1)
        {
            float maxx = x[0], maxy = y[0];
            float minx = x[0], miny = y[0];
            if (x.size() != y.size())
            {
                throw std::runtime_error("x and y size not equal");
            }
            for(int i = 1; i < x.size(); i++)
            {
                if (x[i] > maxx) maxx = x[i];
                if (y[i] > maxx) maxx = y[i];
                if (x[i] < minx) minx = x[i];
                if (y[i] < minx) minx = y[i];
            }
            cv::Mat img(cv::Size(800, 800), CV_8UC3, cv::Scalar(255, 255, 255));
            for (int i = 0; i < x.size(); i++)
            {
                cv::Point2f p((x[i] - minx) / (maxx - minx) * 800, 800 - ((y[i] - miny) / (maxy - miny) * 800));
                cv::circle(img, p, size, color, -1);
            }
            return img;
        }
    }
}

#endif