#ifndef NAHSOR_HPP
#define NAHSOR_HPP
#include <opencv2/opencv.hpp>
#include "nahsor_modules/nahsor_detector.hpp"
#include "nahsor_modules/nahsor_prediction.hpp"
#include "rm_util/rm_util.hpp"
namespace nahsor
{
    class NahsorAlgo
    {
    public:
        NahsorAlgo(rclcpp::Node::SharedPtr node,
                   std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool);
        int smallPred(cv::Mat &img);
        int bigPred(cv::Mat &img, double t);
        int noPred(cv::Mat &img);
        /**
         * @brief 建立图模型
         * 
         */
        void graph();
        /**
         * @brief 给图添加边
         * 
         * @param angle 角度
         * @param time 时间戳
         */
        void addEdge(double angle, double time);
        /**
         * @brief 进行优化
         * 
         */
        void optimize();
        
        cv::Point2f get_target();

        cv::Point2f get_R_center();
        double get_angle();
    private:
        rclcpp::Node::SharedPtr node_;
        NahsorDetector nahsor_detector;
        std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool_;
        g2o::SparseOptimizer optimizer;
        CurveFittingVertex *v;
        double t0,w,a,b;//v = a * sin(w * t) + b
        int count{0};
        bool build_graph{false};
        double target_x,target_y;
        double delta_t = 0.7;
        double target_angle;
        //上次的时间和角度，用以计算速度
        double last_angle{-1};
        double last_time{-1};
        //是否顺时针旋转
        bool is_clockwise;
        double temp_t{0};
        bool optimized{false};
    };
}
#endif