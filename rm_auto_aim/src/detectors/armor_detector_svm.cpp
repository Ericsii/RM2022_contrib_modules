#include "rm_auto_aim/detector/armor_detector_svm.hpp"

#include <sstream>

#include "rm_util/rm_util.hpp"

using namespace rm_auto_aim;

ArmorDetectorSVM::ArmorDetectorSVM(rclcpp::Node::SharedPtr node,
                                   bool armor_is_red,
                                   const std::string &xml_path) : ArmorDetector(node, armor_is_red)
{
    node_->declare_parameter("xml_path", "");
    std::string config_path = xml_path;
    if (config_path == "")
    {
        config_path = node_->get_parameter("xml_path").as_string();
    }
    if (config_path == "")
    {
        throw std::invalid_argument("SVM xml path cannot be none!");
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Loading SVM model from: %s", config_path.c_str());

    node_->declare_parameter("color_hsv",
                             std::vector<int64_t>(
                                 {150, 100, 100,
                                  170, 255, 255,
                                  0, 50, 100,
                                  60, 255, 255,
                                  50, 50, 100,
                                  120, 255, 255}));
    std::vector<int64_t> color_hsv = node_->get_parameter("color_hsv").as_integer_array();
    this->color_hsv = color_hsv;

    svm_ = cv::ml::SVM::load(config_path);
    // HOG_SVM数字识别参数设置
    std::map<int, int> m_label2id;
    m_label2id = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 11}, {7, 7}, {8, 8}};
    m_hog.winSize = cv::Size(48, 32);
    m_hog.blockSize = cv::Size(16, 16);
    m_hog.blockStride = cv::Size(8, 8);
    m_hog.cellSize = cv::Size(8, 8);
    m_hog.nbins = 9;
    m_hog.derivAperture = 1;
    m_hog.winSigma = -1;
    m_hog.histogramNormType = cv::HOGDescriptor::L2Hys;
    m_hog.L2HysThreshold = 0.2;
    m_hog.gammaCorrection = false;
    m_hog.free_coef = -1.f;
    m_hog.nlevels = cv::HOGDescriptor::DEFAULT_NLEVELS;
    m_hog.signedGradient = false;

    // 构造投影目标点
    float offset_x = 0, offset_y = 0;
    perspective_targets_[0] = cv::Point2f(offset_x, offset_y);
    perspective_targets_[1] = cv::Point2f(offset_x, 32 - offset_y);
    perspective_targets_[2] = cv::Point2f(32 - offset_x, 32 - offset_y);
    perspective_targets_[3] = cv::Point2f(32 - offset_x, offset_y);

    std::string str;
    std::stringstream ss;

    for (int i = 0; i < 4; ++i)
    {
        ss << i << ": " << perspective_targets_ << std::endl;
    }
    ss >> str;
    RCLCPP_INFO(
        node_->get_logger(),
        "Aromor perspective : %s", str.c_str());

    RCLCPP_INFO(
        node_->get_logger(),
        "ArmorDetector initialized. Target color: %s.",
        target_color_red_ ? "red" : "blue");
}

ArmorDetectorSVM::~ArmorDetectorSVM()
{
    svm_.release();
    armors_.~vector();
    lights_.~vector();
    contours_.~vector();
    hierarchy_.~vector();
}

std::vector<ArmorDescriptor> &ArmorDetectorSVM::getArmorVector()
{
    return armors_;
}

int ArmorDetectorSVM::preImg(cv::Mat &src, cv::Mat &dst)
{
    cv::Mat bgr[3];
    cv::Mat img_b, img_g, img_r;
    cv::split(src, bgr);
    img_b = bgr[0];
    img_g = bgr[1];
    img_r = bgr[2];
    cv::Mat imgTargetColor, imgTargetColor_temp, imgBrightness, imgTargetColorRedHigh, imgTargetColorRedLow;

    if (target_color_red_)
    {
        imgTargetColor_temp = img_r - color_hsv[14] * 0.1 * img_g - color_hsv[15] * 0.1 * img_b;
        cv::inRange(src, cv::Scalar(color_hsv[0], color_hsv[1], color_hsv[2]), cv::Scalar(color_hsv[3], color_hsv[4], color_hsv[5]), imgTargetColorRedHigh);
        cv::inRange(src, cv::Scalar(color_hsv[6], color_hsv[7], color_hsv[8]), cv::Scalar(color_hsv[9], color_hsv[10], color_hsv[11]), imgTargetColorRedLow);
        cv::add(imgTargetColorRedHigh, imgTargetColorRedLow, imgTargetColor);
        cv::bitwise_and(imgTargetColor, imgTargetColor_temp, imgTargetColor);
        imgTargetColor = imgTargetColor_temp;
        imgBrightness = img_r;
    }
    else
    {
        imgTargetColor = img_b - color_hsv[12] * 0.1 * img_r - color_hsv[13] * 0.1 * img_g;
        // cv::inRange(src, cv::Scalar(color_hsv[12], color_hsv[13], color_hsv[14]), cv::Scalar(color_hsv[15], color_hsv[16], color_hsv[17]), imgTargetColor);
        // cv::bitwise_and(imgTargetColor, imgTargetColor_temp, imgTargetColor);
        imgBrightness = img_b;
    }

    //目标颜色区域
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(color_hsv[16], color_hsv[16]));
    cv::threshold(imgTargetColor, imgTargetColor, 100, 255, cv::THRESH_BINARY);
    cv::erode(imgTargetColor, imgTargetColor, element);
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(color_hsv[17], color_hsv[17]));
    cv::dilate(imgTargetColor, imgTargetColor, element);
#ifdef RM_DEBUG_MODE
    cv::imshow("target_color_dialte_thre", imgTargetColor);
#endif

    //亮度区域
    cv::GaussianBlur(imgBrightness, imgBrightness, cv::Size(3, 3), 1);
    cv::threshold(imgBrightness, imgBrightness, 100, 255, cv::THRESH_BINARY);
#ifdef RM_DEBUG_MODE
    cv::imshow("bright_thre", imgBrightness);
#endif

    //逻辑与，求交集
    cv::bitwise_and(imgTargetColor, imgBrightness, dst);
#ifdef RM_DEBUG_MODE
    cv::imshow("bitand_dst", dst);
#endif

    cv::blur(dst, dst, cv::Size(3, 3));
    cv::medianBlur(dst, dst, 3);
#ifdef RM_DEBUG_MODE
    cv::imshow("dst", dst);
#endif

    return 0;
}

int ArmorDetectorSVM::getLightDescriptor(cv::RotatedRect r, LightDescriptor &light)
{
    light.lightArea = r.size.area();
    if (light.lightArea < 50 || light.lightArea > 50000)
    {
        return 1; // 灯条区域大小不满足
    }
    light.box = r;
    cv::Point2f rect_points[4];
    r.points(rect_points);
    float lineLen1 = cv::norm(rect_points[0] - rect_points[1]);
    float lineLen2 = cv::norm(rect_points[1] - rect_points[2]);
    cv::Point2f endPoint1, endPoint2; // 直线端点
    if (lineLen1 > lineLen2)
    {
        // line1为长边
        endPoint1 = (rect_points[0] + rect_points[3]) / 2.;
        endPoint2 = (rect_points[1] + rect_points[2]) / 2.;
        light.lightHeight = lineLen1;
        light.lightWidth = lineLen2;
    }
    else
    {
        // line2为长边
        endPoint1 = (rect_points[0] + rect_points[1]) / 2;
        endPoint2 = (rect_points[2] + rect_points[3]) / 2;
        light.lightHeight = lineLen2;
        light.lightWidth = lineLen1;
    }
    light.lightRatioWH = light.lightWidth / light.lightHeight;
    /*
    area:       195  133    65    41   44      1025   2970
    height:     25   21     13    10   8.8     73     124
    wh:         0.3  0.27   0.3   0.5  0.57    0.19   0.19
    sqrt(h)*wh  1.5  1.23   1.1   1.58 1.69    1.65   2.11    取3
    */
    if (light.lightRatioWH * sqrt(light.lightHeight) > 5)
    {
        return 21; //宽高比不符合，与区域面积（高度）有关，正相关
    }
    if (light.lightRatioWH > 1)
    {
        return 2; //宽高比不符合，与区域面积（高度）有关，正相关
    }

    //直线的上下端点
    if (endPoint1.y > endPoint2.y)
    {
        light.led_bottom = endPoint1;
        light.led_top = endPoint2;
    }
    else
    {
        light.led_bottom = endPoint2;
        light.led_top = endPoint1;
    }

    light.lightAngle = rm_util::calc_inclination_angle(light.led_top, light.led_bottom);

    if (abs(light.lightAngle - 90) > 45)
    {
        return 3; //灯条倾斜度不符合
    }

    light.led_center = r.center;

    return 0;
}

int ArmorDetectorSVM::lightsMatch(LightDescriptor &l1, LightDescriptor &l2, ArmorDescriptor &armor)
{
    if (l1.led_center.x > l2.led_center.x)
    {
        armor.lightR = &l1;
        armor.lightL = &l2;
    }
    else
    {
        armor.lightR = &l2;
        armor.lightL = &l1;
    }
    armor.parallelErr = fabs(armor.lightL->lightAngle - armor.lightR->lightAngle);
    armor.armorWidth = cv::norm(armor.lightL->led_center - armor.lightR->led_center);
    armor.armorHeight = armor.lightL->lightHeight > armor.lightR->lightHeight ? armor.lightL->lightHeight : armor.lightR->lightHeight; //取两灯条最大值
    armor.armorRatioWH = armor.armorWidth / armor.armorHeight;
    // 条件0 两边灯条大小比较
    if (fabs(l1.lightArea / l2.lightArea) < 0.3 || fabs(l1.lightArea / l2.lightArea) > 3)
    {
        return -1; // 两边灯条大小不匹配
    }
    //条件1
    if (armor.armorHeight < 60)
    {
        if (armor.parallelErr > 5)
        {
            return 11; //平行太大误差，否定
        }
    }
    else
    {
        if (armor.parallelErr > armor.armorHeight / 30)
        {
            return 12; //平行太大误差，否定
        }
        if(armor.lightL->lightAngle>90 && armor.lightR->lightAngle<90)
            return 1;
        if(armor.lightL->lightAngle<90 && armor.lightR->lightAngle>90)
            return 1;
    }
    if (armor.parallelErr > 5)
    {
        if (armor.lightL->lightAngle >= 90 && armor.lightR->lightAngle <= 90)
            return 13;
        if (armor.lightL->lightAngle <= 90 && armor.lightR->lightAngle >= 90)
            return 14;
    }
    //条件2
    if (armor.armorRatioWH < 1 || armor.armorRatioWH > (22.5 / 6))
    {
        return 2; //宽高比不符合，否定
    }

    float innerAngle;
    if (armor.lightL->lightHeight > armor.lightR->lightHeight)
    {
        innerAngle = rm_util::calc_inner_angle(
            armor.lightL->led_center,
            armor.lightL->led_top, armor.lightR->led_center);
    }
    else
    {
        innerAngle = rm_util::calc_inner_angle(
            armor.lightR->led_center,
            armor.lightR->led_top, armor.lightL->led_center);
    }
    armor.innerAngleErr = abs(rm_util::rad_to_deg(innerAngle) - 90);
    //条件3
    if (armor.innerAngleErr > 30)
    {
        return 3; //装甲板正度（矩形内角=90）不符合，否定
    }
    armor.horizonLineAngle = rm_util::calc_inclination_angle(
        armor.lightL->led_center,
        armor.lightR->led_center);
    if (armor.horizonLineAngle > 180)
        armor.horizonLineAngle = armor.horizonLineAngle - 360;
    //条件4
    if (abs(armor.horizonLineAngle) > 30)
    {
        return int(abs(armor.horizonLineAngle)); //装甲板倾斜不符合，否定
    }
    //修补
    if (armor.lightL->lightHeight > armor.lightR->lightHeight)
    {
        lightHeightLong(armor.lightR, armor.armorHeight); //修补右灯条
    }
    else
    {
        lightHeightLong(armor.lightL, armor.armorHeight); //补偿左灯条
    }
    //计算中心点
    armor.centerPoint = (armor.lightR->led_center + armor.lightL->led_center) / 2;

    //装甲板四个点
    armor.points[0] = armor.lightL->led_top;
    armor.points[1] = armor.lightL->led_bottom;
    armor.points[2] = armor.lightR->led_bottom;
    armor.points[3] = armor.lightR->led_top;

    return 0;
}

int ArmorDetectorSVM::process(cv::Mat &src)
{
    // 清空上一次信息
    lights_.clear();
    armors_.clear();
    contours_.clear();
    hierarchy_.clear();
#ifdef RM_DEBUG_MODE
    cv::waitKey(1);
#endif

    preImg(src, binImg_);

    // 提取灯条
    cv::findContours(binImg_, contours_, hierarchy_,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    int ret, lightCnt = 0;
    cv::RotatedRect r_rect;
    LightDescriptor light;

#ifdef RM_DEBUG_MODE
    RCLCPP_INFO(
        node_->get_logger(),
        "contour size: %lu", contours_.size());
#endif

    for (const auto &contour : contours_)
    {
        if (contour.size() > 5)
        {
            r_rect = cv::fitEllipse(contour);
            ret = getLightDescriptor(r_rect, light);
            // std::cout << "light: " << ret << std::endl;
            if (!ret)
            {
                light.id = lightCnt++;
                lights_.push_back(light);
            }
        }
    }

#ifdef RM_DEBUG_MODE
    cv::Mat debugImg = src.clone();
    for (const auto &light : lights_)
    {
        rm_util::draw_rotated_rect(debugImg, light.box);
        cv::putText(debugImg,
                    std::to_string(int(light.lightAngle)),
                    light.led_bottom,
                    cv::FONT_HERSHEY_SIMPLEX, 1,
                    target_color_red_ ? rm_util::red : rm_util::red, 5);
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "light size: %lu", lights_.size());
#endif

    // 匹配灯条
    if (lights_.size() < 2)
    {
        return 1; // 灯条数目不够
    }
    // 遍历匹配
    ArmorDescriptor t_armor;
    for (size_t i = 0; i < lights_.size(); ++i)
    {
        for (size_t j = i + 1; j < lights_.size(); ++j)
        {
            ret = lightsMatch(lights_[i], lights_[j], t_armor);
            // std::cout << "lightsMatch: " << ret << std::endl;
            if (!ret)
            {
                ret = getArmorNumber(src, t_armor);
            }
            if (!ret)
            {
                armors_.push_back(t_armor);
            }
        }
    }

#ifdef RM_DEBUG_MODE
    RCLCPP_INFO(
        node_->get_logger(),
        "armor size: %lu", armors_.size());
#endif

    if (armors_.size() < 1)
    {
        return 2; // 无装甲板
    }
#ifdef RM_DEBUG_MODE
    std::vector<ArmorDescriptor>::iterator armor = armors_.begin();
    for (; armor != armors_.end(); armor++)
    {
        // if (armor->label == 0)
        // {
        // std::vector<ArmorDescriptor>::iterator tmp = armor;
        // armor = armors_.erase(tmp);
        // continue;
        // }
        rm_util::draw_4points(debugImg, armor->points);
        cv::putText(debugImg,
                    std::to_string(armor->label),
                    armor->points[0],
                    cv::FONT_HERSHEY_SIMPLEX, 1,
                    target_color_red_ ? rm_util::red : rm_util::blue, 5);
        cv::circle(debugImg, armor->centerPoint, 5, {0, 255, 0}, 3);
    }
    cv::imshow("result", debugImg);
#endif
    if (armors_.size() < 1)
    {
        return 2; // 无装甲板
    }
    return 0;
}

void ArmorDetectorSVM::lightHeightLong(LightDescriptor *light, float height)
{
    float coff; //补偿比例系数
    coff = (float)((height - light->lightHeight) / height * 0.7);
    cv::Point2f dVec = coff * (light->led_bottom - light->led_top);
    light->led_bottom = light->led_bottom + dVec;
    light->led_top = light->led_top - dVec;
}

int ArmorDetectorSVM::getArmorNumber(cv::Mat &src, ArmorDescriptor &armor)
{
    // 调整数字识别区域
    ArmorDescriptor armor_num(armor);
    cv::Point2f dVecnx(armor.armorWidth * 0.2, 0), dVecny(0, armor.armorHeight * 0.2);
    //装甲板四个点
    armor_num.points[0] = armor.lightL->led_top + dVecnx - dVecny;
    armor_num.points[1] = armor.lightL->led_bottom + dVecnx + dVecny;
    armor_num.points[2] = armor.lightR->led_bottom - dVecnx + dVecny;
    armor_num.points[3] = armor.lightR->led_top - dVecnx - dVecny;
    auto t_M = cv::getPerspectiveTransform(armor_num.points, perspective_targets_);           // 计算投影矩阵
    cv::warpPerspective(src, transformImg_, t_M, cv::Size(32, 32));                           // 投影变换
    cv::cvtColor(transformImg_, transformImg_, cv::COLOR_BGR2GRAY);                           // 转换灰度图
    cv::threshold(transformImg_, transformImg_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // 二值化
    cv::resize(transformImg_, transformImg_, cv::Size(48, 32));

#ifdef RM_DEBUG_MODE
    cv::imshow("armor_num", transformImg_);
#endif

    std::vector<float> descriptors;
    m_hog.compute(transformImg_, descriptors, cv::Size(8, 8));
    armor.label = svm_->predict(descriptors); // 预测图片
    if (armor.label >= 0 && armor.label <= 8)
    {
        // 检测成功
        return 0;
    }
    return -1;
}
