#include "rm_sentry/opencv_detector.hpp"
namespace rm_sentry
{
    int argmax(const float *ptr, int len)
    {
        int max_arg = 0;
        for (int i = 1; i < len; i++)
        {
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        return max_arg;
    }
    double get_max(double b1, double b2, double b3, double b4)
    {
        double max_value = b1;
        max_value = std::max(max_value, b2);
        max_value = std::max(max_value, b3);
        max_value = std::max(max_value, b4);
        return max_value;
    }

    double get_min(double b1, double b2, double b3, double b4)
    {
        double min_value = b1;
        min_value = std::min(min_value, b2);
        min_value = std::min(min_value, b3);
        min_value = std::min(min_value, b4);
        return min_value;
    }

    bool is_overlap(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2)
    {
        cv::Rect2f bbox1, bbox2;
        bbox1.x = get_min(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x);
        bbox1.y = get_min(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y);
        bbox1.width = get_max(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x) - bbox1.x;
        bbox1.height = get_max(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y) - bbox1.y;
        bbox2.x = get_min(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x);
        bbox2.y = get_min(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y);
        bbox2.width = get_max(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x) - bbox2.x;
        bbox2.height = get_max(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y) - bbox2.y;
        return (bbox1 & bbox2).area() > 0;
    }

    double sigmoid(float x)
    {
        return 1 / (1 + std::exp(-x));
    }
    Detector::Detector(std::string xml_path, std::string bin_path, float threhold)
    {

        net = cv::dnn::readNetFromModelOptimizer(xml_path, bin_path);
        _cof_threshold = threhold;
    }
    Detector::~Detector() {}

    bool Detector::process_frame(cv::Mat &img, std::vector<Bbox> &detected_objects)
    {
        std::vector<Bbox> raw_detected;
        float scale = 640.f / std::max(img.cols, img.rows);
        cv::Mat temp;

        cv::resize(img, temp, {(int)round(img.cols * scale), (int)round(img.rows * scale)});

        cv::Mat input(640, 640, CV_8UC3, 127);
        temp.copyTo(input({0, 0, temp.cols, temp.rows}));

        auto x = cv::dnn::blobFromImage(input);

        net.setInput(x);
        auto y = net.forward();
        // std::cout << y.size << std::endl;
        for (int i = 0; i < y.size[1]; i++)
        {
            float *result = (float *)y.data + i * y.size[2];

            if (sigmoid(result[8]) < _cof_threshold)
                continue;
            Bbox box;
            for (int k = 0; k < 4; k++)
            {
                box.pts.push_back({result[k * 2 + 0] / scale, result[k * 2 + 1] / scale});
            }

            box.color_id = argmax(result + 9, 4);
            box.tag_id = argmax(result + 13, 9);

            box.conf = sigmoid(result[8]);

            raw_detected.push_back(box);
        }
        std::sort(raw_detected.begin(), raw_detected.end(), [](Bbox a, Bbox b) -> bool
                  { return a.conf > b.conf; });
        std::vector<bool> is_removed(raw_detected.size());
        for (size_t i = 0; i < raw_detected.size(); i++)
        {
            if (is_removed[i])
                continue;
            detected_objects.push_back(raw_detected[i]);
            for (size_t j = i + 1; j < raw_detected.size(); j++)
            {
                if (is_removed[j])
                    continue;
                if (is_overlap(raw_detected[i].pts, raw_detected[j].pts))
                    is_removed[j] = true;
            }
        }
        return true;
    }
}