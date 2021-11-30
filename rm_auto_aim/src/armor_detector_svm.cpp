//HitArmorData ArmorDetectorSVM::GetTargetArmor(cv::Mat src)

#include "rm_auto_aim/armor_detector_svm.hpp"

using namespace rm_auto_aim;

inline bool CompareLed(Led a, Led b)
{
    return a.box.center.x < b.box.center.x;
}
inline double GetDistance(cv::Point2f a, cv::Point2f b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
inline float GetArmorAngle(cv::Point2f p[4])
{
    return ((atan2(p[0].y - p[1].y, p[1].x - p[0].x) + atan2(p[3].y - p[2].y, p[2].x - p[3].x)) / 2);
}
inline cv::Point2f GetArmorCenter(cv::Point2f p[4])
{
    return cv::Point2f((p[0].x + p[1].x + p[2].x + p[3].x) / 4,
                       (p[0].y + p[1].y + p[2].y + p[3].y) / 4);
}

std::vector<ArmorData> ArmorDetectorSVM::GetArmors(cv::Mat &src)
{
    cv::Mat gray;
    PreDeal(src, gray);
    std::vector<ArmorData> armors;

    if (GetLeds(gray) && GetArmor(src))
    {
        //成功识别到目标
        armors = armor_datas_;
        leds_.clear();
        armor_datas_.clear();
    }
    else
    {
        armors = armor_datas_;
        leds_.clear();
        armor_datas_.clear();
    }
    return armors;
}

void ArmorDetectorSVM::PreDeal(cv::Mat &src, cv::Mat &gray)
{
    cv::Mat bgr[3];
    cv::Mat img_b, img_g, img_r;
    split(src, bgr);
    img_b = bgr[0];
    img_g = bgr[1];
    img_r = bgr[2];
    cv::Mat imgTargetColor, imgBrightness;

    if (armor_is_red_)
    {
        imgTargetColor = img_r - 0.2 * img_g - 0.8 * img_b;
        imgBrightness = img_r;
    }
    else
    {
        imgTargetColor = img_b - 0.8 * img_r - 0.2 * img_g;
        imgBrightness = img_b;
    }
    //目标颜色区域
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    dilate(imgTargetColor, imgTargetColor, element);
    threshold(imgTargetColor, imgTargetColor, 72, 255, cv::THRESH_BINARY);
    //亮度区域
    GaussianBlur(imgBrightness, imgBrightness, cv::Size(3, 3), 1);
    threshold(imgBrightness, imgBrightness, 150, 255, cv::THRESH_BINARY);
    //逻辑与，求交集
    bitwise_and(imgTargetColor, imgBrightness, gray);

    blur(gray, gray, cv::Size(3, 3));
    medianBlur(gray, gray, 3);
}

bool ArmorDetectorSVM::GetLeds(cv::Mat &gray)
{
    std::vector<std::vector<cv::Point>> gray_contours; //存储灰度轮廓，用于处理，获得完整灯条
    cv::findContours(gray, gray_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < gray_contours.size(); i++)
    {
        if (gray_contours[i].size() < 5)
            continue;
        cv::RotatedRect box = fitEllipse(gray_contours[i]);
        //根据角度信息筛选
        if (fabs(box.angle - 90) < 45)
            continue;
        //根据长宽比例筛选
        if (box.size.height / box.size.width > 15 || box.size.height / box.size.width < 1.3)
            continue;
        Led led_new;
        led_new.box = box;
        int flag = 1;
        if (box.angle > 90)
            flag = -1;
        led_new.led_top = cv::Point2f(box.center.x + flag * box.size.height * sin(box.angle * M_PI / 180) / 2.0,
                                      box.center.y - flag * box.size.height * cos(box.angle * M_PI / 180) / 2.0);
        led_new.led_bottom = cv::Point2f(box.center.x - flag * box.size.height * sin(box.angle * M_PI / 180) / 2.0,
                                         box.center.y + flag * box.size.height * cos(box.angle * M_PI / 180) / 2.0);
        leds_.push_back(led_new);
    }
    if (leds_.size() >= 2)
        return true;
    else
    {
#ifdef DEBUG_MODE
        std::cout << "No leds!" << std::endl;
#endif
        return false;
    }
}

bool ArmorDetectorSVM::GetArmor(cv::Mat &src)
{
    //将内部灯条按中心x坐标升序排列
    sort(leds_.begin(), leds_.end(), CompareLed);
    for (size_t i = 0; i < leds_.size(); i++)
    {
        for (size_t j = i + 1; j < leds_.size(); j++)
        {
            //过大就终止循环
            if (leds_[j].box.center.x - leds_[i].box.center.x > leds_[i].box.size.height * 5)
                break;
            //根据角度筛选
            if ((leds_[i].box.angle < 90 && leds_[j].box.angle < 90) || (leds_[i].box.angle > 90 && leds_[j].box.angle > 90))
            {
                if (fabs(leds_[i].box.angle - leds_[j].box.angle) > 10)
                    continue;
            }
            else if (leds_[i].box.angle < 90 && leds_[j].box.angle > 90)
            {
                if (leds_[i].box.angle - leds_[j].box.angle > -173)
                    continue;
            }
            else if (leds_[i].box.angle > 90 && leds_[j].box.angle < 90)
            {
                if (leds_[j].box.angle - leds_[i].box.angle > -173)
                    continue;
            }
            else
                continue;
            //根据高度差筛选，极值15
            double height_cha_max = 15 > (leds_[i].box.size.height + leds_[j].box.size.height) / 10
                                        ? (leds_[i].box.size.height + leds_[j].box.size.height) / 10
                                        : 15;
            if (fabs(leds_[i].box.size.height - leds_[j].box.size.height) > height_cha_max)
                continue;
            //根据宽度差筛选,极值15
            double width_on = GetDistance(leds_[i].led_top, leds_[j].led_top);
            double width_down = GetDistance(leds_[j].led_bottom, leds_[i].led_bottom);
            double width_cha_max = 15 > (width_down + width_on) / 15
                                       ? (width_down + width_on) / 15
                                       : 15;
            if (fabs(width_down - width_on) > width_cha_max)
                continue;
            //根据平行角度差筛选,将y差值的符号给x可得装甲角得到灯条角,并排除水平0度
            double angle_armor = atan2(fabs(leds_[i].led_top.y - leds_[j].led_top.y),
                                       (leds_[j].led_top.x - leds_[i].led_top.x)) *
                                 180 / M_PI;
            if ((leds_[i].box.angle > 90 && leds_[j].box.angle < 90) || (leds_[i].box.angle < 90 && leds_[j].box.angle > 90))
            {
                //防止内八或外八
                if (angle_armor > 8)
                    continue;
            }
            else if (leds_[i].box.angle > 90 && leds_[j].box.angle > 90)
            {
                if ((angle_armor + (leds_[i].box.angle + leds_[j].box.angle) / 2) < 162 || (angle_armor + (leds_[i].box.angle + leds_[j].box.angle) / 2) > 210)
                    continue;
            }
            else if (leds_[i].box.angle < 90 && leds_[j].box.angle < 90)
            {
                if (fabs(angle_armor - (leds_[i].box.angle + leds_[j].box.angle) / 2) > 18)
                    continue;
            }
            else
            {
                if (angle_armor > 7)
                    continue;
            }
            //定义装甲对象
            ArmorData Armor;
            //根据比例筛选
            double propotion = ((width_on + width_down) / 2) / ((leds_[i].box.size.height + leds_[j].box.size.height) / 2);
            //高比x差值
            if (propotion >= small_min_ratio && propotion <= small_max_ratio)
            {
                Armor.is_small = true;
            }
            else if (propotion >= big_min_ratio && propotion <= big_max_ratio)
            {
                Armor.is_small = false;
            }
            else
            {
                //不符合比例条件
                continue;
            }
            //筛选完毕将点存入,由于已排序，必定i在左侧
            Armor.point[0] = leds_[i].led_top;
            Armor.point[1] = leds_[j].led_top;
            Armor.point[2] = leds_[j].led_bottom;
            Armor.point[3] = leds_[i].led_bottom;
            //传参
            Armor.leds_[0] = leds_[i];
            Armor.leds_[1] = leds_[j];
            //装甲确定完毕
            armor_datas_.push_back(Armor);
        }
    }
    //进入数字检测
    for (size_t i = 0; i < armor_datas_.size(); i++)
    {

        float angle = GetArmorAngle(armor_datas_[i].point);
        int ArmorNumber = GetNumber(src,
                                    GetArmorCenter(armor_datas_[i].point),
                                    (armor_datas_[i].leds_[0].box.size.height + armor_datas_[i].leds_[1].box.size.height) / 2,
                                    angle);
        armor_datas_[i].armor_num = ArmorNumber;
        return true;
    }
    //判断是否有装甲板
    if (armor_datas_.size() < 1)
    {
#ifdef DEBUG_MODE
        std::cout << "NO armor!" << std::endl;
#endif
        return false;
    }
    else
    {
        return true;
    }
}

int ArmorDetectorSVM::GetNumber(cv::Mat &src, cv::Point2f center, float height, float angle)
{
    float y_top = center.y - height;
    float y_bottom = center.y + height;
    float x_left = center.x - 0.7 * height;
    float x_right = center.x + 0.7 * height;

    if (angle > 90)
    {
        angle = -(180 - angle);
    }
    if (y_top < 0)
    {
        y_top = 0;
    }
    if (y_bottom > src.rows)
    {
        y_bottom = src.rows;
    }
    if (x_left < 0)
    {
        x_left = 0;
    }
    if (x_right > src.cols)
    {
        x_right = src.cols;
    }

    cv::Mat armor_roi = src(cv::Rect(x_left, y_top, x_right - x_left, y_bottom - y_top)); //截取装甲板区域
    cvtColor(armor_roi, armor_roi, cv::COLOR_BGR2GRAY);                                   //彩色图片转换为灰度图
    resize(armor_roi, armor_roi, cv::Size(28, 28));                                       //将图片大小缩放为28×28
    armor_roi.convertTo(armor_roi, CV_32F);                                               //更换数据类型有uchar->float32
    normalize(armor_roi, armor_roi, 1, 0, cv::NORM_MINMAX);                               //归一化
    armor_roi = armor_roi.reshape(1, 1);                                                  //单通道，一行
    int num = svm_->predict(armor_roi);                                                   //预测图片
    return num;
}
