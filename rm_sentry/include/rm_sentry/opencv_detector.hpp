#ifndef OPENCV_DETECTOR_H
#define OPENCV_DETECTOR_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/dnn/dnn.hpp>
#include <cmath>
#include <algorithm>

namespace rm_sentry
{
    struct Bbox
    {
        int id;
        int tag_id;
        int color_id;
        float conf;
        std::vector<cv::Point2f> pts;
    };
    class Detector
    {
    public:
        Detector(std::string xml_path, std::string bin_path, float threhold);
        ~Detector();
        //初始化
        // bool init(std::string xml_path, double cof_threshold);
        //释放资源
        // bool uninit();
        //处理图像获取结果
        bool process_frame(cv::Mat &inframe, std::vector<Bbox> &detected_objects);

        // bool is_overlap(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2);

        // bool init(std::string xml_path, std::string bin_path);

    private:
        // double sigmoid(double x);
        // std::vector<int> get_anchors(int net_grid);

        // bool parse_yolov5(cv::Mat &img, std::vector<Object>& raw_detected);
        // cv::Rect detet2origin(const cv::Rect& dete_rect,float rate_to,int top,int left);
        //存储初始化获得的可执行网络

        std::string _input_name;
        //参数区
        std::string _xml_path;
        std::string _bin_path; // OpenVINO模型xml文件路径
        double _cof_threshold;
        cv::dnn::Net net;
    };
}
#endif