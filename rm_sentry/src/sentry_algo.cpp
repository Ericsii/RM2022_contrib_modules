#include "rm_sentry/sentry_algo.hpp"
namespace rm_sentry
{
    double max_dis = 20000.0;
    bool debug = true;
    SentryAlgo::SentryAlgo(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool,
                           bool target_color_red,
                           std::string &xml_path,
                           std::string &bin_path,
                           double threhold) : node_(node),
                                              mono_location_tool_(mono_location_tool),
                                              sentry_detector_(node, target_color_red, xml_path, bin_path, threhold)
    {
        // RCLCPP_INFO(node_->get_logger(), "algo init");
        float realWidth, realHeight, half_x, half_y;

        // 小装甲
        realHeight = 6.3;
        realWidth = 12.8;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mSmallArmorPoints.emplace_back(-half_x, half_y, 0);
        mSmallArmorPoints.emplace_back(-half_x, -half_y, 0);
        mSmallArmorPoints.emplace_back(half_x, -half_y, 0);
        mSmallArmorPoints.emplace_back(half_x, half_y, 0);

        // 大装甲
        realWidth = 22.2;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mBigArmorPoints.emplace_back(-half_x, half_y, 0.);
        mBigArmorPoints.emplace_back(-half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, half_y, 0);

        Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
        z_k(0, 0) = 0;
        z_k(1, 0) = 0;
        z_k(2, 0) = 0;

        ekf_filter = new rm_filters::ExKalman(
            rm_filters::MState::const_acc, rm_filters::MState::df_const_acc,
            rm_filters::MState::se_df_const_acc, rm_filters::MState::const_acc_sensor,
            rm_filters::MState::df_const_acc_sensor, rm_filters::MState::se_df_const_acc_sensor);
        ekf_filter->init(z_k);
        U = Eigen::MatrixXd(rm_filters::Matrix_x, rm_filters::Matrix_x);
        for (int i = 0; i <= 5; i++)
        {
            for (int j = 0; j <= 5; j++)
            {
                U(i, j) = 0;
            }
        }
        // 相机到陀螺仪旋转
        Eigen::Matrix3d rotation;
        rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        cam2imu_static_ = Eigen::Quaterniond(rotation);
        RCLCPP_INFO(node_->get_logger(), "algo init finish");
    }
    int SentryAlgo::process(double time_stamp_ms, cv::Mat &src, Eigen::Quaterniond pose)
    {
        // RCLCPP_INFO(node_->get_logger(), "algo process");
        //(void)time_stamp_ms;
        bool same_armor = false, same_id = false;
        double initial_vel = 30., shoot_delay = 0.110;
        using std::sort;
        using std::vector;
        Eigen::Quaterniond imu_q(pose.w(), pose.x(), pose.y(), pose.z());
        imu_q.normalize();

        auto euler_angles = rm_util::CoordinateTranslation::quat2euler(pose);
        float c_pitch, c_yaw;
        c_pitch = rm_util::rad_to_deg(euler_angles(1));
        c_yaw = rm_util::rad_to_deg(euler_angles(0));
        auto img_center_point = cv::Point2f(src.cols / 2, src.rows / 2);
        cv::Point3f p;
        int res = sentry_detector_.process(src);
        auto armors = sentry_detector_.getArmorVector();
        if (debug)
        {
            cv::Mat temp = src.clone();
            for (auto armor : armors)
            {

                cv::circle(temp, armor.centerPoint, 5, cv::Scalar(255, 0, 0));
            }
            // std::cout << temp.size << std::endl;
            cv::imshow("debug", temp);
            cv::waitKey(1);
        }
        std::vector<bool> same_armor_flag(armors.size());
        std::vector<bool> same_id_flag(armors.size());

        int max_v = 0, idx = -1;
        if (res == 1)
        {

            for (size_t k = 0; k < armors.size(); k++)
            {
                auto armor = armors[k];

                armor_target.armorDescriptor = armor;

                points.clear();
                points.push_back(armor.points[0]);
                points.push_back(armor.points[1]);
                points.push_back(armor.points[2]);
                points.push_back(armor.points[3]);
                mono_location_tool_->solve_pnp(points, mBigArmorPoints,
                                               armor_target.postion, armor_target.rotation);
                p = armor_target.postion;

                Eigen::Vector3d position3d(p.x, p.y, p.z);
                armor_target.distance = position3d.norm();
                same_armor = last_shoot && is_same_armor(last_position3d, position3d);
                same_id = last_shoot && armor.label == last_target.armorDescriptor.label;
                same_armor_flag[k] = same_armor;
                same_id_flag[k] = same_id;
                //装甲板灰色,如果是自己上一次击打的装甲板，则视为敌人，否则放行
                //只有在攻击范围内才会选为目标
                if (armor.label / 9 == 2)
                {
                    if (!same_armor)
                        continue;
                }
                //最高优先级:英雄, 直接攻击
                if (armor.label % 9 == 1)
                {
                    if (armor_target.distance < max_dis)
                    {
                        idx = k;
                        
                        mTarget = armor_target;
                        break;
                    }
                }
                //次高优先级:上次击打的装甲板
                else if (same_armor)
                {
                    if (armor_target.distance < max_dis)
                    {
                        max_v = 4;
                        mTarget = armor_target;
                        idx = k;
                    }
                }
                //次次高优先级，上次正在打的车
                else if (same_id)
                {
                    if (armor_target.distance < max_dis)
                        if (max_v < 3)
                        {
                            max_v = 3;
                            mTarget = armor_target;
                            idx = k;
                        }
                }
                //其他优先级:进入斩杀线的工程以及其他步兵
                else if (armor.label % 9 == 2)
                {
                    // todo: 此处应查询地方工程血量
                    if (false)
                    {
                        if (armor_target.distance < max_dis)
                        {
                            if (max_v < 2)
                            {
                                max_v = 2;
                                mTarget = armor_target;
                                idx = k;
                            }
                        }
                    }
                }
                else
                {
                    if (armor_target.distance < max_dis)
                    {
                        if (max_v < 2)
                        {
                            max_v = 2;
                            mTarget = armor_target;
                            idx = k;
                        }
                    }
                }
            }
        }
        //未发现目标, 摸鱼
        if (idx == -1)
        {
            sentry_detector_.last_shoot = false;
            last_shoot = false;
            last_box.tag_id = -1;
            return -1;
        }
        same_armor = same_armor_flag[idx];
        same_id = same_id_flag[idx];
        //打上次击打的装甲板
        // if (!selected && last_shoot)
        // {
        //     for (auto armor : armors)
        //     {
        //         armor_target.armorDescriptor = armor;
        //         if (armor.label == last_target.armorDescriptor.label)
        //         {
        //             points.clear();
        //             points.push_back(armor.points[0]);
        //             points.push_back(armor.points[1]);
        //             points.push_back(armor.points[2]);
        //             points.push_back(armor.points[3]);
        //             mono_location_tool_->solve_pnp(points, mSmallArmorPoints,
        //                                            armor_target.postion, armor_target.rotation);

        //             Eigen::Vector3d position3d_camera(p.x, p.y, p.z);
        //             Eigen::Vector3d position3d = imu_q * cam2imu_static_ * position3d_camera;

        //             if (is_same_armor(last_position3d, position3d))
        //             {
        //                 mTarget = armor_target;
        //                 selected = true;
        //                 same_armor = true;
        //             }
        //         }
        //     }
        // }
        // //打上次击打的车
        // if (!selected && last_shoot)
        // {
        //     for (auto armor : armors)
        //     {
        //         if (armor.label == last_target.armorDescriptor.label)
        //         {
        //             points.clear();
        //             points.push_back(armor.points[0]);
        //             points.push_back(armor.points[1]);
        //             points.push_back(armor.points[2]);
        //             points.push_back(armor.points[3]);
        //             //太远的不打
        //             mono_location_tool_->solve_pnp(points, mSmallArmorPoints,
        //                                            armor_target.postion, armor_target.rotation);
        //             p = armor_target.postion;
        //             Eigen::Vector3d position3d(p.x, p.y, p.z);
        //             armor_target.distance = position3d.norm();
        //             if (armor_target.distance < max_dis)
        //             {
        //                 mTarget = armor_target;
        //                 selected = true;
        //                 same_id = true;
        //             }
        //         }
        //     }
        // }
        // //没有英雄，也没有上次击打的车，击打离图像中心最近的车
        // if (!selected)
        // {
        //     sort(armors.begin(), armors.end(), [img_center_point](auto &a, auto &b)
        //          {
        //              auto dis_a = cv::norm(img_center_point - a.centerPoint);
        //              auto dis_b = cv::norm(img_center_point - b.centerPoint);
        //              return dis_a < dis_b; });
        //     for (auto armor : armors)
        //     {
        //         armor_target.armorDescriptor = armor;

        //         points.clear();
        //         points.push_back(armor.points[0]);
        //         points.push_back(armor.points[1]);
        //         points.push_back(armor.points[2]);
        //         points.push_back(armor.points[3]);
        //         mono_location_tool_->solve_pnp(points, mSmallArmorPoints,
        //                                        armor_target.postion, armor_target.rotation);
        //         p = armor_target.postion;
        //         Eigen::Vector3d position3d(p.x, p.y, p.z);
        //         armor_target.distance = position3d.norm();
        //         if (armor_target.distance < max_dis)
        //         {
        //             mTarget = armor_target;
        //             selected = true;
        //         }
        //     }
        // }
        //筛选完成，开始瞄准
        double predict_time = mTarget.distance / initial_vel + shoot_delay;

        // 【相机->世界】目标相机坐标系--->在世界坐标系下的坐标（陀螺仪的世界坐标系）
        Eigen::Vector3d position3d_camera(p.x, p.y, p.z);
        Eigen::Vector3d position3d = imu_q * cam2imu_static_ * position3d_camera;
        // debug = false;
        if (debug)
        {
            // RCLCPP_INFO(node_->get_logger(), "[camera] target_x: %f, target_y: %f, target_z: %f", p.x, p.y, p.z);
            // RCLCPP_INFO(node_->get_logger(), "[world] target_x: %f, target_y: %f, target_z: %f", position3d(0, 0), position3d(1, 0), position3d(2, 0));
        }

        double target_pitch = atan2(position3d(2, 0), position3d.topRows<2>().norm());
        double target_yaw = atan2(position3d(1, 0), position3d(0, 0));
        //std::cout << target_pitch << std::endl;
        double target_distance = position3d.norm();
        double target_height = position3d(2, 0);
        if (debug)
        {
            // RCLCPP_INFO(node_->get_logger(), "real_target_pitch: %f, real_target_yaw: %f", target_pitch, target_yaw);
            // RCLCPP_INFO(node_->get_logger(), "target_distance: %f, target_height: %f", target_distance, target_height);
        }
        double pre_target_pitch = target_pitch;
        double pre_target_yaw = target_yaw;
        double pre_target_distance = target_distance;
        double pre_target_height = target_height;
        //std::cout << "height: " << target_height << std::endl;
        double time = time_stamp_ms - last_time;
        mTarget_yaw = float(target_yaw);
        mTarget_pitch = float(target_pitch);

        //同一块装甲板,EKF滤波
        if (same_armor)
        {
            //std::cout << "same armor" << std::endl;
            Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
            z_k(0, 0) = position3d(0, 0), z_k(1, 0) = position3d(1, 0);
            z_k(2, 0) = position3d(2, 0);

            ekf_filter->predict(U, time);
            Eigen::MatrixXd x_k = ekf_filter->update(z_k);

            filter_position3d_world(0, 0) = z_k(0, 0) + x_k(3, 0) * predict_time;
            filter_position3d_world(1, 0) = z_k(1, 0) + x_k(4, 0) * predict_time;
            filter_position3d_world(2, 0) = z_k(2, 0) + x_k(5, 0) * predict_time;
        }

        // if (debug)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "predict_target_pitch: %f, predict_target_yaw: %f",
        //                 target_pitch, target_yaw);
        // }
        else if (same_id)
        {
            //先放着
            // TODO: copy 反陀螺
            // RCLCPP_INFO(node_->get_logger(), "1");
        }
        else
        {
            filter_position3d_world(0, 0) = position3d(0, 0);
            filter_position3d_world(1, 0) = position3d(1, 0);
            filter_position3d_world(2, 0) = position3d(2, 0);
        }

        // 【世界->相机】滤波后目标世界坐标系--->在相机坐标系下的坐标
        Eigen::Vector3d pre_camera;
        pre_camera(0, 0) = filter_position3d_world(0, 0);
        pre_camera(1, 0) = filter_position3d_world(1, 0);
        pre_camera(2, 0) = filter_position3d_world(2, 0);
        pre_camera = cam2imu_static_.inverse() * imu_q.inverse() * pre_camera;

        /* yaw预测补偿*/
        pre_target_yaw = rm_util::rad_to_deg(atan2(pre_camera(0, 0), pre_camera(2, 0)));
        pre_target_distance = pre_camera.norm();

        /* pitch抬枪补偿 distance(单位：mm) height(单位：m)*/
        pre_target_height = filter_position3d_world(2, 0) - 5; //距离枪口高度
        //std::cout << "pre_height: " << pre_target_height << std::endl;
        auto offset_pitch = std::make_shared<rm_trajectory::GetPitch>(initial_vel);
        double pitch_temp = offset_pitch->get_pitch(pre_target_distance * 10, pre_target_height * 10, initial_vel);
        pre_target_pitch = pitch_temp; //此处得到的是绝对姿态pitch，需要减去当前pitch角

        mTarget_pitch = float(pre_target_pitch - c_pitch);
        mTarget_yaw = float(pre_target_yaw);

        if (debug)
        {
            cv::Mat debugImg = src.clone();
            cv::putText(debugImg,
                        std::to_string(int(pre_target_distance)),
                        mTarget.armorDescriptor.points[0],
                        cv::FONT_HERSHEY_SIMPLEX, 1,
                        rm_util::red, 3);
            cv::putText(debugImg,
                        std::to_string(mTarget.armorDescriptor.label),
                        mTarget.armorDescriptor.points[1],
                        cv::FONT_HERSHEY_SIMPLEX, 1,
                        rm_util::blue, 3);
            cv::circle(debugImg, mTarget.armorDescriptor.centerPoint, 5, {0, 255, 0}, 3);
            cv::cv2eigen(mono_location_tool_->camera_intrinsic_, F);
            Eigen::Vector3d pre_img = F * pre_camera / pre_camera(2, 0);
            cv::circle(debugImg, {int(pre_img(0, 0)), int(pre_img(1, 0))}, 5, {255, 255, 0}, 3);
            cv::imshow("target", debugImg);
            cv::waitKey(1);
        }

        //假设发弹了
        last_time = time_stamp_ms;
        // last_label = mTarget.armorDescriptor.label;
        last_shoot = true;
        last_target = mTarget;
        last_position3d = position3d;
        return 0;
    }

    bool SentryAlgo::is_same_armor(Eigen::Vector3d old_position3d, Eigen::Vector3d now_position3d)
    {
        double distance = (now_position3d - old_position3d).norm();

        if (distance < 1000)
            return true;
        else
            return false;
    }

    rm_auto_aim::ArmorTarget SentryAlgo::getTarget()
    {
        return mTarget;
    }
}
