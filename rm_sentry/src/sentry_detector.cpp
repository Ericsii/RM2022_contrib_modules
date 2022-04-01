#include "rm_sentry/sentry_detector.hpp"
#include "rm_util/rm_util.hpp"
#include <chrono>
namespace rm_sentry
{
    cv::Point2f points_center(std::vector<cv::Point2f> pts)
    {
        assert(pts.size() == 4);
        for (int i = 0; i < 4; ++i)
        {
            for (int j = i + 1; j < 4; ++j)
            {
                if (pts[i] == pts[j])
                {
                    std::cout << "[Error] Unable to calculate center point." << std::endl;
                    return cv::Point2f{0, 0};
                }
            }
        }
        cv::Point2f center(0, 0);
        if (pts[0].x == pts[2].x && pts[1].x == pts[3].x)
        {
            std::cout << "[Error] Unable to calculate center point." << std::endl;
        }
        else if (pts[0].x == pts[2].x)
        {
            center.x = pts[0].x;
            center.y = (pts[3].y - pts[1].y) / (pts[3].x - pts[1].x) * (pts[0].x - pts[3].x) + pts[3].y;
        }
        else if (pts[1].x == pts[3].x)
        {
            center.x = pts[1].x;
            center.y = (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x) * (pts[1].x - pts[0].x) + pts[0].y;
        }
        else
        {
            center.x = (((pts[3].y - pts[1].y) / (pts[3].x - pts[1].x) * pts[3].x - pts[3].y +
                         pts[0].y - (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x) * pts[0].x)) /
                       ((pts[3].y - pts[1].y) / (pts[3].x - pts[1].x) - (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x));
            center.y = (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x) * (center.x - pts[0].x) + pts[0].y;
        }

        return center;
    }
    int max_dead_buffer = 20;
    SentryDetector::SentryDetector(rclcpp::Node::SharedPtr node,
                                   bool target_color_red, std::string &xml_path, std::string &bin_path, double threhold) : ArmorDetector(node, target_color_red), module(xml_path, bin_path, threhold)
    {
    }
    int SentryDetector::process(cv::Mat &img)
    {
        exist_hero = false;
        detections.clear();
        auto start_time = std::chrono::high_resolution_clock::now();
        module.process_frame(img, detections);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end_time - start_time;

        // RCLCPP_INFO(node_->get_logger(), "find %ld cars, use %d s", detections.size(),diff.count());

        int dead_buffer = 0;
        int target_color = target_color_red_ ? 1 : 0;
        bool flag[6];
        cv::Mat temp = img.clone();
        armors.clear();
        //剔除我方车，不打的车和已经没血的车
        for (size_t k = 0; k < detections.size(); k++)
        {
            auto d = detections[k];
            for (int i = 0; i < 4; i++)
            {
                cv::line(temp, d.pts[i], d.pts[(i + 1) % 4], cv::Scalar(0, 255, 0));
            }
            if (d.color_id == target_color)
            {
                armors.push_back(bbox2Armor(d));
                
            }
            else if (d.color_id == 2 && last_shoot && last_box.tag_id == d.tag_id)
            {
                
                armors.push_back(bbox2Armor(d));
            }

            // RCLCPP_INFO(node_->get_logger(), "id: %d color_id:%d cof:%lf", d.tag_id, d.color_id, d.conf);
        }
        
        cv::imshow("ddd", temp);
        cv::waitKey(1);
        return armors.size() > 0 ? 1 : 0;
    }
    std::vector<rm_auto_aim::ArmorDescriptor> &SentryDetector::getArmorVector()
    {
        return armors;
    }

    rm_auto_aim::ArmorDescriptor SentryDetector::bbox2Armor(Bbox box)
    {
        rm_auto_aim::ArmorDescriptor armor;
        rm_auto_aim::LightDescriptor *lightL = new rm_auto_aim::LightDescriptor(), *lightR = new rm_auto_aim::LightDescriptor();
        for (int i = 0; i < 4; i++)
        {
            armor.points[i] = box.pts[i];
        }
        armor.centerPoint = points_center(box.pts);
        lightL->led_bottom = box.pts[1];
        lightL->led_top = box.pts[0];
        lightR->led_bottom = box.pts[2];
        lightR->led_top = box.pts[3];
        armor.lightL = lightL;
        armor.lightR = lightR;
        armor.label = box.color_id * 9 + box.tag_id;
        
        return armor;
    }

}
