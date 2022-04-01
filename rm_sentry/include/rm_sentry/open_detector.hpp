#ifndef OPEN_DETECTOR_H
#define OPEN_DETECTOR_H
#include <opencv2/opencv.hpp>
#include <inference_engine.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/dnn/dnn.hpp>
#include <cmath>
#include <algorithm>
using namespace std;
using namespace cv;
using namespace InferenceEngine;

namespace rm_sentry
{
    struct Bbox
    {
        int id;
        int tag_id;
        int color_id;
        float cof;
        vector<Point2f> pts;
    };
    class Detector
    {
    public:
        Detector(string xml_path, double cof_threshold);
        ~Detector();
        /**
         * @brief 初始化openvino检测器
         * 
         * @param xml_path xml文件路径
         * @param cof_threshold 置信度阈值
         * @return true 初始化成功
         * @return false 
         */
        bool init(string xml_path, double cof_threshold);
        //释放资源
        bool uninit();
        //处理图像获取结果
        bool process_frame(Mat &inframe, vector<Bbox> &detected_objects);

        bool is_overlap(vector<Point2f> pts1, vector<Point2f> pts2);

    private:
        double sigmoid(double x);
        vector<int> get_anchors(int net_grid);
        bool parse_yolov5(const Blob::Ptr &blob, int net_grid, float cof_threshold,
                          vector<Bbox> &raw_detected);
        Rect detet2origin(const Rect &dete_rect, float rate_to, int top, int left);
        //存储初始化获得的可执行网络
        ExecutableNetwork _network;
        OutputsDataMap _outputinfo;
        string _input_name;
        //参数区
        string _xml_path;      // OpenVINO模型xml文件路径
        double _cof_threshold; //置信度阈值,计算方法是框置信度乘以物品种类置信度
        // double _nms_area_threshold;  //nms最小重叠面积阈值
    };
}
#endif