#include "rm_hero_auto_aim/auto_aim_algo.hpp"

#include <iostream>
#include <algorithm>
#include <Eigen/Core>

#include "rm_hero_auto_aim/armor_detector_svm.hpp"

namespace rm_hero_auto_aim
{
    AutoAimAlgo::AutoAimAlgo(rclcpp::Node::SharedPtr node,
                             std::vector<double> camera_intrinsic,
                             std::vector<double> camera_distortion,
                             std::shared_ptr<ArmorDetector> armor_detector) : node_(node), armor_detector_(armor_detector)
    {
        mono_loacation_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_intrinsic, camera_distortion);

        float realWidth, realHeight, half_x, half_y;

        // 小装甲
        realHeight = 6.3;
        realWidth = 12.8;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        m_small_armor_points_.emplace_back(-half_x, half_y, 0.);
        m_small_armor_points_.emplace_back(-half_x, -half_y, 0);
        m_small_armor_points_.emplace_back(half_x, -half_y, 0);
        m_small_armor_points_.emplace_back(half_x, half_y, 0);

        // 大装甲
        realWidth = 22.2;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        m_big_armor_points_.emplace_back(-half_x, half_y, 0.);
        m_big_armor_points_.emplace_back(-half_x, -half_y, 0);
        m_big_armor_points_.emplace_back(half_x, -half_y, 0);
        m_big_armor_points_.emplace_back(half_x, half_y, 0);

        mono_loacation_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_intrinsic, camera_distortion);

        Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
        z_k(0, 0) = 0;
        z_k(1, 0) = 0;
        z_k(2, 0) = 0;

        
        ekf_filter = new rm_filters::ExKalman(
                rm_filters::MState::const_acc, rm_filters::MState::df_const_acc, 
                rm_filters::MState::se_df_const_acc, rm_filters::MState::const_acc_sensor, 
                rm_filters::MState::df_const_acc_sensor, rm_filters::MState::se_df_const_acc_sensor);
        ekf_filter->init(z_k);
    }

    int AutoAimAlgo::process(double time_stamp, cv::Mat &src, Eigen::Quaterniond pose)
    {
        // 自瞄预测参数
        bool same_armor = false, same_id = false;
        double initial_vel = 30., shoot_delay = 0.110;

        // 仅供测试使用，无弹道补偿和滤波跟踪
        using std::vector;
        using std::sort;

        (void) time_stamp;
        (void) pose;
        
        // 陀螺仪四元数、坐标系转换四元数
        Eigen::Quaterniond imu_q(pose.w(), pose.x(), pose.y(), pose.z());
        Eigen::Quaterniond q(imu_q.matrix().transpose()); //    imu_q四元数矩阵的逆矩阵

        auto img_center_point = cv::Point2f(src.cols / 2, src.rows / 2);
        // step1.图像识别
        int ret;
        ret = armor_detector_->process(src);
        if (ret != 0)
        {
            cv::Mat debugImg = src;
            cv::imshow("target", debugImg);
            return 1; // 无目标
        }
        auto armor_descriptors = armor_detector_->getArmorVector();

#ifdef RM_DEBUG_MODE
    RCLCPP_INFO(
        node_->get_logger(),
        "detected armor size: %d", armor_descriptors.size());
#endif
        
        // 按照图像中心远近排序
        sort(armor_descriptors.begin(), armor_descriptors.end(), [img_center_point](const ArmorDescriptor &a, const ArmorDescriptor &b){
            auto dis_a = cv::norm(img_center_point - a.centerPoint);
            auto dis_b = cv::norm(img_center_point - b.centerPoint);
            return dis_a < dis_b;
        });

        armors_centers_.push_back(armor_descriptors.front().centerPoint);

        if(guard_mode_)
        {
            /*击打哨兵的逻辑*/
            attackGuard();
        }

        // step2.位置解算
        vector<ArmorTarget> armor_target_vec;
        for (size_t i = 0; i < armor_descriptors.size(); ++i)
        {
            ArmorTarget armor_target;
            armor_target.armorDescriptor = armor_descriptors[i];
            auto label = armor_descriptors[i].label;
            vector<cv::Point2f> points;
            points.push_back(armor_descriptors[i].points[0]);
            points.push_back(armor_descriptors[i].points[1]);
            points.push_back(armor_descriptors[i].points[2]);
            points.push_back(armor_descriptors[i].points[3]);

            if (label == 0 || label == 1 || label == 8)
            {
                // 大装甲板匹配
                mono_loacation_tool_->solve_pnp(points, m_big_armor_points_,
                                                armor_target.postion, armor_target.rotation);   
                armor_target.isBigArmor = true;
            }
            else
            {
                // 小装甲板匹配
                mono_loacation_tool_->solve_pnp(points, m_small_armor_points_,
                                                armor_target.postion, armor_target.rotation);
            }
            armor_target_vec.push_back(armor_target);
        }

        // step3.筛选出当前最重要目标
        // 1）判断是否为上次击打同一装甲板
        
        // 2）判断是否为同一id的车     
        // 3）判断是否为哨兵（仅左右运动）
        // 4）判断是否为英雄
        //TODO:目标筛选

        // 目标在世界坐标系下的坐标（陀螺仪的世界坐标系）
        cv::Point3f p = armor_target_vec[0].postion;
        Eigen::Vector3d position3d(p.x, p.y, p.z);
        position3d = q * position3d;

        double predict_time = position3d.norm() / initial_vel + shoot_delay;
        if(1)
        {
            RCLCPP_INFO(node_->get_logger(),"[camera] target_x: %f, target_y: %f, target_z: %f", p.x, p.y, p.z);
            RCLCPP_INFO(node_->get_logger(),"[world] target_x: %f, target_y: %f, target_z: %f", position3d(0,0),position3d(1,0),position3d(2,0));
        }

        // step4.滤波预测补偿
        double target_pitch = atan2(position3d(2, 0), position3d.topRows<2>().norm());
        double target_yaw = atan2(position3d(1, 0), position3d(0, 0));
        double target_distance = position3d.norm();
        double target_height = position3d(2, 0);

        if(1)
        {
            RCLCPP_INFO(node_->get_logger(),"real_target_pitch: %f, real_target_yaw: %f", target_pitch, target_yaw);
            RCLCPP_INFO(node_->get_logger(),"target_distance: %f, target_height: %f", target_distance, target_height);
        }

        /* pitch抬枪补偿*/
        auto offset_pitch = std::make_shared<rm_trajectory::GetPitch>(initial_vel);
        target_pitch = offset_pitch->get_pitch(target_distance, target_height, initial_vel);

        // 击打同一块装甲板，EKF滤波预测
        if (same_armor)
        {
            Eigen::MatrixXd U(rm_filters::Matrix_x, rm_filters::Matrix_x);
            U(0, 0) = 0; U(0, 1) = 0;  U(0, 5) = 0;
	        U(0, 2) = 0; U(0, 3) = 0;  U(0, 4) = 0;
	        U(1, 0) = 0; U(1, 1) = 0;
	        U(1, 2) = 0; U(1, 3) = 0;  U(1, 4) = 0; U(1, 5) = 0;
	        U(2, 0) = 0; U(2, 1) = 0;  U(2, 5) = 0;
	        U(2, 2) = 0; U(2, 3) = 0;  U(2, 4) = 0;
	        U(3, 0) = 0; U(3, 1) = 0;  U(3, 5) = 0;
	        U(3, 2) = 0; U(3, 3) = 0;  U(3, 4) = 0;
        	U(4, 0) = 0; U(4, 1) = 0;  U(4, 5) = 0;
	        U(4, 2) = 0; U(4, 3) = 0;  U(4, 4) = 0;
            U(5, 0) = 0; U(5, 1) = 0;  U(5, 2) = 0; U(5, 3) = 0;
            U(5, 4) = 0; U(5, 5) = 0;

            Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
            z_k(0, 0) = position3d(0, 0), z_k(1, 0) = position3d(1, 0);
            z_k(2, 0) = position3d(2, 0);
            
            this->ekf_filter->predict(U, predict_time);
            Eigen::MatrixXd x_k = this->ekf_filter->update(z_k);                       
        }

        if(1)
        {
            // RCLCPP_INFO(node_->get_logger(),"x_v: %f, y_v: %f", x_k(4,0), x_k(5,0));
            RCLCPP_INFO(node_->get_logger(),"predict_target_pitch: %f, predict_target_yaw: %f", target_pitch, target_yaw);
            RCLCPP_INFO(node_->get_logger(),"predict_target_pitch: %f, predict_target_yaw: %f", target_pitch, target_yaw);
        }
        
        if (same_id)
        {
            //反小陀螺模式：同一id，不同装甲
            switch(judgeRotation())
            {
                case 0: m_target_.armorDescriptor = armor_descriptors.front(); break;
                case 1: m_target_.armorDescriptor = armor_descriptors.front();/*m_target_.armorDescriptor = 预判后的装甲板信息；*/ break;
                case 2: this->last_yaw = target_yaw; m_target_.armorDescriptor = armor_descriptors.front();/*m_target_.armorDescriptor = 当前指向位置，即不变；*/ break;
                default: this->last_yaw = target_yaw; m_target_.armorDescriptor = armor_descriptors.front();/*m_target_.armorDescriptor = 当前指向位置，即不变；*/ break;
            }
            //TODO::EKF滤波
        }
        return 0;
    }

    int AutoAimAlgo::judgeRotation()
    {
        int size = armors_centers_.size();
        // 当有效装甲板数满足时计算中心装甲板跳跃次数
        if(size > effective_armors_quantity_)
        {
            double distances_sum(0);
            for(int i(0); i < (size - 1); i++)
            {
                distances_sum += cv::norm(armors_centers_[i] - armors_centers_[i + 1]);
            }
            double average_distance = distances_sum / size;

            int jump_count(0);
            for(int i(0); i < (size - 1); i++)
            {
                // 如果间隔大于5倍平均距离则认为该间隔发生了一次跳跃
                if(cv::norm(armors_centers_[i] - armors_centers_[i + 1]) > (5 * average_distance))
                {
                    jump_count++;
                }
            }

            armors_centers_.erase(armors_centers_.end());

            if(jump_count > fast_rotate_jump_count_) 
                return 2;
            else if(jump_count > 0 && jump_count < fast_rotate_jump_count_)
                return 1;
            else
                return 0;
        }
        else
            return 0;
    }

    void AutoAimAlgo::attackGuard()
    {

    }

    ArmorTarget AutoAimAlgo::getTarget()
    {
        return this->m_target_;
    }

    void AutoAimAlgo::setTargetColor(bool is_red)
    {
        armor_detector_->set_target_color(is_red);
    }

    void AutoAimAlgo::setTrack(bool is_track)
    {
        m_is_track_ = is_track;
    }
}