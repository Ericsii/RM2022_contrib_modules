//HitArmorData ArmorDetectorSVM::GetTargetArmor(Mat src)

#include "rm_auto_aim/armor_detector_svm.hpp"

using namespace rm_auto_aim;

inline bool CompareLed(Led a,Led b)
{
    return a.box.center.x<b.box.center.x;
}
inline double GetDistance(Point2f a,Point2f b)
{
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}
inline float GetArmorAngle(Point2f p[4])
{
     return((atan2(p[0].y - p[1].y,p[1].x - p[0].x)+atan2(p[3].y - p[2].y,p[2].x - p[3].x))/2);
}
inline Point2f GetArmorCenter(Point2f p[4])
{
    return Point2f((p[0].x+p[1].x+p[2].x+p[3].x)/4,(p[0].y+p[1].y+p[2].y+p[3].y)/4);
}
inline bool CompareArmor(ArmorData a,ArmorData b)
{
    return (a.leds_[0].box.size.height+a.leds_[1].box.size.height)>(b.leds_[0].box.size.height+b.leds_[1].box.size.height);
}

ArmorData ArmorDetectorSVM::GetTargetArmor(Mat& src)
{
    //(void)timestamp_ms;
    Mat gray;
    PreDeal(src,gray);

    ArmorData best_armor_data;                                    //记录识别数据

    if(GetLeds(gray) && GetArmor() && GetBestArmor(src,best_armor_data))
    {
        //成功识别到目标
        lost_number = 0;
        armor_old_ = best_armor_data;
        leds_.clear();
        armor_datas_.clear();
    }
    else
    {
        //未识别到目标
        if(++lost_number < LOST_MAX)
        {                   //缓冲
            best_armor_data = armor_old_;
            best_armor_data.statu = buffering;
        }
        else
        {                                                                        //掉帧
            best_armor_data.statu = stop;
            shoot_armor_number_ = -1;
            armor_old_.point[0] = Point2f(0,0);
        }
        leds_.clear();
        armor_datas_.clear();
    }
    return best_armor_data;
}

void ArmorDetectorSVM::PreDeal(Mat& src, Mat& gray)
{
    Mat bgr[3];
    Mat img_b, img_g, img_r;
    split(src, bgr);   //将三个通道的像素值分离
    img_b = bgr[0];
    img_g = bgr[1];
    img_r = bgr[2];
    Mat imgTargetColor, imgBrightness;
    // default taget color is red
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
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    dilate(imgTargetColor, imgTargetColor, element);
    threshold(imgTargetColor, imgTargetColor, 72, 255, cv::THRESH_BINARY);
    //亮度区域
    GaussianBlur(imgBrightness, imgBrightness, Size(3, 3), 1);
    threshold(imgBrightness, imgBrightness, 150, 255, cv::THRESH_BINARY);
    //逻辑与，求交集
    bitwise_and(imgTargetColor, imgBrightness, gray);

    blur(gray,gray,Size(3,3));
    medianBlur(gray,gray,3);

    //imshow("a",gray);
}

bool ArmorDetectorSVM::GetLeds(Mat& gray)
{
    std::vector<std::vector<cv::Point>>gray_contours;                       //存储灰度轮廓，用于处理，获得完整灯条
    cv::findContours(gray,gray_contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);

    for(size_t i = 0;i<gray_contours.size();i++)
    {
        if(gray_contours[i].size()<5)continue;
        RotatedRect box = fitEllipse(gray_contours[i]);
        //根据角度信息筛选
        if(fabs(box.angle - 90)<45)continue;
        //根据长宽比例筛选
        if(box.size.height/box.size.width>15||box.size.height/box.size.width<1.3)continue;
        Led led_new;
        led_new.box = box;
        int fuhao = 1;
        if(box.angle>90)
            fuhao = -1;
        led_new.led_top = Point2f(box.center.x + fuhao*box.size.height*sin(box.angle*M_PI/180)/2.0,box.center.y - fuhao*box.size.height*cos(box.angle*M_PI/180)/2.0);
        led_new.led_bottom = Point2f(box.center.x - fuhao*box.size.height*sin(box.angle*M_PI/180)/2.0,box.center.y + fuhao*box.size.height*cos(box.angle*M_PI/180)/2.0);
        leds_.push_back(led_new);
    }
    if(leds_.size()>=2)
        return true;
    else
    {
        cout<<"灯条少于2"<<endl;
        return false;
    }
}

bool ArmorDetectorSVM::GetArmor()
{
    //将内部灯条按中心x坐标升序排列
    sort(leds_.begin(),leds_.end(),CompareLed);
    for(size_t i = 0;i<leds_.size();i++)
    {
        for(size_t j = i+1;j<leds_.size();j++)
        {
            //过大就终止循环
            if(leds_[j].box.center.x - leds_[i].box.center.x>leds_[i].box.size.height*5)break;
            //根据角度筛选
            if((leds_[i].box.angle<90&&leds_[j].box.angle<90)||(leds_[i].box.angle>90&&leds_[j].box.angle>90))
            {
                if(fabs(leds_[i].box.angle - leds_[j].box.angle)>10)continue;
            }
            else if(leds_[i].box.angle<90&&leds_[j].box.angle>90)
            {
                if(leds_[i].box.angle - leds_[j].box.angle > -173)continue;
            }
            else if(leds_[i].box.angle>90&&leds_[j].box.angle<90)
            {
                if(leds_[j].box.angle - leds_[i].box.angle > -173)continue;
            }
            else continue;


            //根据高度差筛选，极值15
            double height_cha_max = 15>(leds_[i].box.size.height + leds_[j].box.size.height)/10?(leds_[i].box.size.height + leds_[j].box.size.height)/10:15;
            if(fabs(leds_[i].box.size.height - leds_[j].box.size.height)>height_cha_max)continue;

            //根据宽度差筛选,极值15
            double width_on = GetDistance(leds_[i].led_top,leds_[j].led_top);
            double width_down = GetDistance(leds_[j].led_bottom,leds_[i].led_bottom);
            double width_cha_max = 15>(width_down + width_on)/15?(width_down + width_on)/15:15;
            if(fabs(width_down - width_on)>width_cha_max)continue;
            //根据平行角度差筛选,将y差值的符号给x可得装甲角得到灯条角,并排除水平0度
            double angle_armor = atan2(fabs(leds_[i].led_top.y - leds_[j].led_top.y),(leds_[j].led_top.x - leds_[i].led_top.x))*180/M_PI;
            if((leds_[i].box.angle>90&&leds_[j].box.angle<90)||(leds_[i].box.angle<90&&leds_[j].box.angle>90))
            {
                //防止内八或外八
                if(angle_armor>8)continue;
            }
            else if(leds_[i].box.angle>90&&leds_[j].box.angle>90)
            {
                if((angle_armor + (leds_[i].box.angle + leds_[j].box.angle)/2)<162||(angle_armor + (leds_[i].box.angle + leds_[j].box.angle)/2)>210)continue;
            }
            else if(leds_[i].box.angle<90&&leds_[j].box.angle<90)
            {
                if(fabs(angle_armor - (leds_[i].box.angle + leds_[j].box.angle)/2)>18)continue;
            }
            else
            {
                if(angle_armor>7)continue;
            }
            //定义装甲对象
            ArmorData Armor;
            //根据比例筛选
            double bili = ((width_on + width_down)/2)/((leds_[i].box.size.height + leds_[j].box.size.height)/2);
            //高比x差值
            if(bili>=small_min_ratio&&bili<=small_max_ratio)
            {
                Armor.is_small = true;
            }
            else if(bili>=big_min_ratio&&bili<=big_max_ratio)
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
    if(armor_datas_.size()<1)
    {
        cout<<"装甲数小于1个"<<endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool ArmorDetectorSVM::GetBestArmor(Mat& src,ArmorData &best_armor)
{
    //判断是否仅识别到一个装甲
    if(armor_datas_.size() == 1)
    {
        if(armor_old_.point[0].x == 0&&armor_old_.point[0].y == 0)
        {
            best_armor.statu = first_find;
        }
        else if(GetDistance(armor_old_.point[0],armor_datas_[0].point[0])<MIN_DISTANCE)
        {
            best_armor.statu = shoot;
        }

        //进入数字检测
        float angle = GetArmorAngle(best_armor.point);
        int ArmorNumber = GetNumber(src,GetArmorCenter(armor_datas_[0].point),(armor_datas_[0].leds_[0].box.size.height+armor_datas_[0].leds_[1].box.size.height)/2,angle);
        armor_datas_[0].armor_num = ArmorNumber;
        if(ArmorNumber == -1)
            return false;
        if(ArmorNumber == 2)
        {
            cout<<"发现工程"<<endl;
            return false;
        }
        else if(ArmorNumber == shoot_armor_number_)
        {
            best_armor.statu = shoot;
        }
        else
        {
            shoot_armor_number_ = ArmorNumber;
            best_armor.statu = first_find;
        }

        best_armor = armor_datas_[0];
        armor_old_ = best_armor;
        return true;
    }
    //第一次识别，前为中断状态
    if(armor_old_.point[0].x == 0&&armor_old_.point[0].y == 0)
    {
        //按装甲高度排序
        sort(armor_datas_.begin(),armor_datas_.end(),CompareArmor);
        for(size_t i = 0;i<armor_datas_.size();i++)
        {
            //进入数字检测
            float angle = GetArmorAngle(best_armor.point);
            int ArmorNumber = GetNumber(src,GetArmorCenter(armor_datas_[i].point),(armor_datas_[i].leds_[0].box.size.height+armor_datas_[i].leds_[1].box.size.height)/2,angle);
            armor_datas_[0].armor_num = ArmorNumber;
            if(ArmorNumber == -1)
                continue;
            if(ArmorNumber == 2)
            {
                cout<<"发现工程"<<endl;
                continue;
            }
            //传参
            best_armor = armor_datas_[i];
            armor_old_ = best_armor;
            best_armor.statu = first_find;
            shoot_armor_number_ = ArmorNumber;
            return true;
        }
        return false;
    }
    //之前已存在目标,打分
    float score_max = SCORE_BEGAIN;            //记录分数极值，SCORE_MIN为标志最小值
    int Index =  -1;                                               //存储最高分序号
    for(size_t i = 0;i<armor_datas_.size();i++)
    {
         float score = 100 - (GetDistance(armor_old_.point[0] ,armor_datas_[i].point[0] )/armor_datas_[i].leds_[0].box.size.height)*10;
        score += 100 - (fabs(armor_old_.leds_[0].box.size.height -armor_datas_[i].leds_[0].box.size.height )/armor_old_.leds_[0].box.size.height)*100;
        if(score>=score_max)
        {
            score_max = score;
            Index = i;
        }
    }
    if(Index!=-1)
    {
        //找到符合标志的目标
        best_armor.statu = shoot;
        best_armor = armor_datas_[Index];
        armor_old_ = best_armor;
        return true;
    }
    else
    {
        //未找到，按装甲高度排序,发现数字相同则取连续,否则取最大装甲
        sort(armor_datas_.begin(),armor_datas_.end(),CompareArmor);
        int max_index = -1;
        int number = 0;
        for(size_t i = 0;i<armor_datas_.size();i++)
        {
            //进入数字检测
            float angle = GetArmorAngle(best_armor.point);
            int ArmorNumber = GetNumber(src,GetArmorCenter(armor_datas_[i].point),(armor_datas_[i].leds_[0].box.size.height+armor_datas_[i].leds_[1].box.size.height)/2,angle);
            armor_datas_[0].armor_num = ArmorNumber;
            if(ArmorNumber == -1)
                continue;
            if(ArmorNumber == 2)
            {
                cout<<"发现工程"<<endl;
                continue;
            }
            if(max_index == -1)
            {
                max_index = i;
                number = ArmorNumber;
            }
            if(ArmorNumber == shoot_armor_number_)
            {
                best_armor.statu = shoot;
                best_armor = armor_datas_[Index];
                armor_old_ = best_armor;
                return true;
            }
        }
        if(max_index != -1)
        {
            best_armor.statu = first_find;
            best_armor = armor_datas_[Index];
            armor_old_ = best_armor;
            shoot_armor_number_ = number;
            return true;
        }
        return false;
    }
}

int ArmorDetectorSVM::GetNumber(Mat& src,Point2f center,float height,float angle)
{

    float y_top = center.y - height;
    float y_bottom = center.y  + height;

    float x_left = center.x - 0.7*height;
    float x_right = center.x + 0.7*height;

    if(angle>90)
    {
        angle = -(180 - angle);
    }
    if(y_top<0)
    {
        y_top = 0;
    }
    if(y_bottom>src.rows)
    {
        y_bottom = src.rows;
    }
    if(x_left<0)
    {
        x_left = 0;
    }
    if(x_right > src.cols)
    {
        x_right = src.cols;
    }

    Mat armor_roi = src(Rect(x_left,y_top,x_right - x_left,y_bottom - y_top));
    //imshow("b",armor_roi);

    cvtColor(armor_roi, armor_roi, cv::COLOR_BGR2GRAY);
    threshold(armor_roi, armor_roi, 300 , 255, cv::THRESH_OTSU);
    //将图片大小缩放为28×28
    resize(armor_roi, armor_roi,cv::Size(28,28));
	//更换数据类型有uchar->float32
	armor_roi.convertTo(armor_roi, CV_32F);
	//归一化
	normalize(armor_roi,armor_roi,1,0,cv::NORM_MINMAX);

	//imshow("c",armor_roi);
	//(1,784)
	armor_roi = armor_roi.reshape(1, 1);
	//预测图片
	int num = svm_->predict(armor_roi);
	return num;
}

