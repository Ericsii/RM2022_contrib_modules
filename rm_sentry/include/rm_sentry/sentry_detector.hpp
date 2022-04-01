#ifndef SENTRY_DETECTOR_HPP
#define SENTRY_DETECTOR_HPP

#include "rm_auto_aim/armor_detector_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

#include "rm_sentry/opencv_detector.hpp"

namespace rm_sentry
{
    class SentryDetector : public rm_auto_aim::ArmorDetector
    {
    public:
        SentryDetector(rclcpp::Node::SharedPtr node,
                       bool target_color_red, std::string &xml_path, std::string &bin_path, double threhold);
        int process(cv::Mat &img);
        std::vector<rm_auto_aim::ArmorDescriptor> &getArmorVector();
        rm_auto_aim::ArmorDescriptor bbox2Armor(Bbox box);
        bool last_shoot;
        Bbox last_box;

    private:
        bool debug{true};
        std::vector<rm_auto_aim::ArmorDescriptor> armors;
        bool exist_hero{false};
        Detector module;
        std::vector<Bbox> detections;
    };
}
#endif