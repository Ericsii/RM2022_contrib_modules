
#include "nahsor_modules/nahsor_detector.hpp"

namespace nahsor
{
    const bool USE_HSV = true;
    bool debug = false;
    const int CORE_SIZE = 3;
    const int MIN_AREA = 1000;
    //std::vector<int> low_red = {156, 180, 50};
    std::vector<int> low_red = {156, 100, 130};
    std::vector<int> up_red = {180, 255, 255};
    std::vector<int> low_red2 = {0, 100, 130};
    std::vector<int> up_red2 = {34, 255, 255};
    std::vector<int> low_blue = {80, 150, 180};
    std::vector<int> up_blue = {124, 255, 255};
    std::vector<std::size_t> armor_size_thre_ = {1450, 1700};
    std::vector<size_t> R_size_thre_ = {100, 200};
    double max_r_angle_ = 0.3;
    std::vector<double> armor_width_thre_ = {0, 100};
    std::vector<double> armor_height_thre_ = {0, 100};
    std::vector<double> armor_area_thre_ = {0, 100};
    std::vector<double> armor_ratio_thre_ = {1.0, 2.0};
    //主逻辑
    NahsorDetector::NahsorDetector(
        rclcpp::Node::SharedPtr node) : node_(node)
    {
        node_->declare_parameter("color", "red");
        color_ = node_->get_parameter("color").as_string();
        // node_->declare_parameter("debug", false);
        // debug_ = node_->get_parameter("debug").as_bool();
        node_->declare_parameter("USE_HSV", true);
        USE_HSV_ = node_->get_parameter("USE_HSV").as_bool();
        //颜色阈值
        // node_->declare_parameter<decltype(red_thre_)>("red_thre");
        // red_thre_ = node_->get_parameter("red_thre").as_integer_array();
        // node_->declare_parameter<decltype(red_thre_)>("blue_thre_");
        // blue_thre_ = node_->get_parameter("blue_thre").as_integer_array();
        // node_->declare_parameter<decltype(red_thre_)>("red2_thre");
        // red2_thre_ = node_->get_parameter("red2_thre").as_integer_array();
        //大小阈值
        // node_->declare_parameter<decltype(red_thre_)>("armor_size_thre");
        // armor_size_thre_ = node_->get_parameter("armor_size_thre").as_integer_array();
        // node_->declare_parameter<decltype(red_thre_)>("R_size_thre");
        // R_size_thre_ = node_->get_parameter("R_size_thre").as_integer_array();
        // node_->declare_parameter<double>("max_r_angle", 0);
        // max_r_angle_ = node_->get_parameter("max_r_angle").as_double();
        // 形状阈值
        // node_->declare_parameter("armor_width_thre_", 0);
        // armor_width_thre_ = node->get_parameter("armor_width_thre_").as_double_array();
        // node_->declare_parameter("armor_height_thre_", 100);
        // armor_height_thre_ = node->get_parameter("armor_height_thre_").as_double_array();
        // node_->declare_parameter("armor_area_thre_", 200);
        // armor_area_thre_ = node->get_parameter("armor_area_thre_").as_double_array();
        // node_->declare_parameter("armor_ratio_thre_", 1.8);
        // armor_ratio_thre_ = node->get_parameter("armor_ratio_thre_").as_double_array();
    }

    //图片预处理
    int NahsorDetector::preProcess(cv::Mat &src,
                                   cv::Mat &dst)
    {
        cv::Mat cvt_img;
        cv::Mat mask;
        if (this->USE_HSV_)
        {
            std::vector<int> lower, upper;
            cv::cvtColor(src, cvt_img, cv::COLOR_BGR2HSV);
            if (color_ == "red")
            {
                cv::inRange(cvt_img, low_red, up_red, mask);
            }
            else
            {
                cv::inRange(cvt_img, low_blue, up_blue, mask);
            }
            if (this->color_ == "red")
            {
                cv::Mat mask_r2;
                cv::inRange(cvt_img, low_red2, up_red2, mask_r2);
                mask = mask + mask_r2;
            }
        }

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(CORE_SIZE, CORE_SIZE));
        cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));

        cv::dilate(mask, mask, erode_kernel);
        cv::erode(mask, mask, erode_kernel);

        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, dst, cv::MORPH_CLOSE, kernel);
        if (debug)
        {
            //RCLCPP_INFO(node_->get_logger(), "debug!");
            cv::namedWindow("mask", 0);
            cv::resizeWindow("mask", int(1200 * (800 - 80) / 800), 800 - 80);

            cv::imshow("mask", dst);
        }
        cv::waitKey(1);
        return 0;
    }

    void NahsorDetector::detect(cv::Mat &img)
    {
        cv::Mat mask;
        int res = preProcess(img, mask);
        if (res == -1)
            return;
        armors_.clear();
        R_edge_.clear();
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        //#ifdef RM_DEBUG_MODE
        if (debug)
        {
            cv::Mat orig1;
            img.copyTo(orig1);

            for (size_t i = 0; i < contours.size(); i++)
            {

                if (cv::contourArea(contours[i]) < 100 or cv::contourArea(contours[i]) > 200)
                    continue;
                // if(i == 2 or i == 8 or i == 12)
                //     RCLCPP_INFO(node_->get_logger(), "%f", cv::contourArea(contours[hierarchy[i][3]]));
                cv::RotatedRect rect = cv::minAreaRect(contours[i]);
                rm_util::draw_rotated_rect(orig1, rect);
                cv::Point2f box[4];
                rect.points(box);
                cv::putText(
                    orig1,
                    std::to_string(contourArea(contours[i])),
                    cv::Point(box[0].x, box[0].y),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1,
                    cv::Scalar(0, 255, 0),
                    2);
            }
            cv::namedWindow("contours", 0);
            cv::resizeWindow("contours", int(1200 * (800 - 80) / 800), 800 - 80);
            cv::imshow("contours", orig1);
        }
        //#endif                 // DEBUG_MODE
        int index = 0; //装甲板对应轮廓下标
        for (size_t i = 0; i < contours.size(); i++)
        {

            if (cv::contourArea(contours[i]) > R_size_thre_[0] && cv::contourArea(contours[i]) < R_size_thre_[1])
            {
                //RCLCPP_INFO(node_->get_logger(), "111");
                R_edge_.push_back(cv::minAreaRect(contours[i]));
            }
            //不含子轮廓
            if (hierarchy[i][2] != -1)
            {
                continue;
            }
            //含有父轮廓
            if (hierarchy[i][3] == -1)
            {
                continue;
            }
            else
            {
                //父轮廓不能再有父轮廓

                cv::Vec4i parent_hierarchy = hierarchy[hierarchy[i][3]];
                if (parent_hierarchy[3] != -1 || cv::contourArea(contours[hierarchy[i][3]]) > 3500)
                {
                    continue;
                }
            }

            if (cv::contourArea(contours[i]) > 100 && cv::contourArea(contours[i]) < 4000)
            {
                cv::RotatedRect armorRect = cv::minAreaRect(cv::Mat(contours[i]));
                Rune armor;
                int ret = 0;
                ret = getArmorDescriptor(armorRect, armor);
                //(node_->get_logger(), "%d", ret);
                if (ret == 0)
                {
                    armors_.push_back(armor);
                    index = i;
                    if (debug)
                    {
                        //RCLCPP_INFO(node_->get_logger(),"detect");
                        cv::Mat img_copy;
                        img.copyTo(img_copy);
                        rm_util::draw_4points(img_copy, armor.points);
                        cv::namedWindow("armor", 0);
                        cv::resizeWindow("armor", int(1200 * (800 - 80) / 800), 800 - 80);
                        cv::imshow("armor", img_copy);
                    }
                }
            }

            cv::waitKey(1);
        }

        //筛完装甲板后,基于装甲板筛选R
        if (armors_.size() == 1)
        {
            //RCLCPP_INFO(node_->get_logger(), "**************");
            auto rect_parent = cv::minAreaRect(contours[hierarchy[index][3]]);
            getR(armors_[0], rect_parent);
            //RCLCPP_INFO(node_->get_logger(), "%f %f %f %f", armors_[0].center.x, armors_[0].center.y, circle.R_center.x, circle.R_center.y);
            //RCLCPP_INFO(node_->get_logger(), "%f", armors_[0].angle);
        }

        for (size_t i = 0; i < armors_.size(); i++)
        {
            armors_[i].id = i;
        }
    }
    int NahsorDetector::getArmorDescriptor(cv::RotatedRect rect,
                                           Rune &armor)
    {

        armor.rect = rect;
        armor.center = rect.center;
        cv::Point2f rect_points[4];
        rect.points(rect_points);
        //确定宽和高，长的边为宽
        float linelen1 = rect.size.height;
        float linelen2 = rect.size.width;
        armor.armor_width = std::max(linelen1, linelen2);
        armor.armor_hight = std::min(linelen1, linelen2);
        bool line1_max_flag = (linelen1 == armor.armor_width);
        armor.armor_area = armor.armor_width * armor.armor_hight;
        armor.armor_ratio_wh = armor.armor_width / armor.armor_hight;
        //删选宽度

        if (armor.armor_width < armor_width_thre_[0] || armor.armor_width > armor_width_thre_[1])
        {
            return 1;
        }
        //筛选高度
        if (armor.armor_hight < armor_height_thre_[0] || armor.armor_hight > armor_height_thre_[1])
        {
            return 2;
        }
        //筛选宽高比
        //RCLCPP_INFO(node_->get_logger(),"%f",armor.armor_ratio_wh);
        if (armor.armor_ratio_wh < armor_ratio_thre_[0] || armor.armor_ratio_wh > armor_ratio_thre_[1])
        {
            return 3;
        }
        cv::Point2f vec_long_line2d, vec_short_line2d;
        if (line1_max_flag)
        {
            vec_long_line2d = rect_points[0] - rect_points[1];
            vec_short_line2d = rect_points[2] - rect_points[1];
        }
        else
        {
            vec_long_line2d = rect_points[2] - rect_points[1];
            vec_short_line2d = rect_points[0] - rect_points[1];
        }
        float z = vec_long_line2d.x * vec_short_line2d.y -
                  vec_short_line2d.x * vec_long_line2d.y;
        if (z > 0)
        {
            // 1做起点
            if (line1_max_flag)
            {
                armor.points[0] = rect_points[1];
                armor.points[1] = rect_points[0];
                armor.points[2] = rect_points[3];
                armor.points[3] = rect_points[2];
            }
            else
            {
                armor.points[0] = rect_points[1];
                armor.points[1] = rect_points[2];
                armor.points[2] = rect_points[3];
                armor.points[3] = rect_points[0];
            }
        }
        else
        {
            // 1不能做起点
            if (line1_max_flag)
            {
                // 0做起点
                armor.points[0] = rect_points[0];
                armor.points[1] = rect_points[1];
                armor.points[2] = rect_points[2];
                armor.points[3] = rect_points[3];
            }
            else
            {
                // 2做起点
                armor.points[0] = rect_points[2];
                armor.points[1] = rect_points[1];
                armor.points[2] = rect_points[0];
                armor.points[3] = rect_points[3];
            }
        }

        return 0;
    }
    void NahsorDetector::getR(Rune &rect_armor, cv::RotatedRect &rect_parent)
    {
        //RCLCPP_INFO(node_->get_logger(), "%d", R_edge_.size());
        auto pLen = std::max(rect_parent.size.width, rect_parent.size.height);
        auto p_center = rect_parent.center;
        for (auto R : R_edge_)
        {
            //RCLCPP_INFO(node_->get_logger(), "**************");
            //RCLCPP_INFO(node_->get_logger(),"%f",R.size);

            auto temp_r_center = R.center;
            auto armor_angle = rm_util::deg_to_rad(rm_util::calc_inclination_angle(temp_r_center, rect_armor.center));
            //筛选距离
            auto ratio = calc_dis(temp_r_center, p_center) / pLen;
            //RCLCPP_INFO(node_->get_logger(),"ratio: %f",ratio);
            if (ratio < 1.3 && ratio > 0.65)
            {
                auto sim = abs(rm_util::deg_to_rad(rm_util::calc_inclination_angle(temp_r_center, p_center)) - armor_angle);
                //RCLCPP_INFO(node_->get_logger(),"sim: %f",sim);
                if (sim < max_r_angle_ ||
                    abs(sim - CV_PI / 2) < max_r_angle_ ||
                    abs(sim - CV_PI) < max_r_angle_)
                {
                    circle.R_center = temp_r_center;
                    circle.radius = calc_dis(temp_r_center, rect_armor.center);
                    rect_armor.angle = armor_angle;
                    return;
                }
            }
        }
        //RCLCPP_INFO(node_->get_logger(), "x:%f, y:%f", circle.R_center.x, circle.R_center.y);
    }
    double NahsorDetector::calc_dis(cv::Point2f &point1, cv::Point2f &point2)
    {
        auto dis = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y);
        return sqrt(dis);
    }
    double NahsorDetector::calc_angle(cv::Point2f &point1, cv::Point2f &point2)
    {
        auto rad = atan2(point2.y - point1.y, point2.x - point1.x);
        return rad;
    }

    std::vector<Rune> NahsorDetector::get_armors()
    {
        return armors_;
    }
    Circle NahsorDetector::get_circle()
    {
        return circle;
    }

}