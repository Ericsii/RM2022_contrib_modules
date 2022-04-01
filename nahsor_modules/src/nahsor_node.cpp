#include "nahsor_modules/nahsor_node.hpp"
namespace nahsor
{

    NahsorNode::NahsorNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("Nahsor_aim", options);
        std::string camera_name = "camera";
        std::string robot_color = "red";
        std::string imu_name = "imu";
        bool auto_start = false;
        bool debug = false;
        //获取参数
        // node_->declare_parameter("robot_color", robot_color);
        node_->declare_parameter("debug", debug);
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("auto_start", auto_start);
        node_->declare_parameter("imu_name", imu_name);
        // node_->get_parameter("robot_color", robot_color);
        node_->get_parameter("debug", debug);
        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("auto_start", auto_start);
        node_->get_parameter("imu_name", imu_name);

        // bool is_red(robot_color == "red");
        using namespace std::placeholders;
        gimbal_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>(
            "/cmd_gimbal", 10);
        set_mode_srv_ = node_->create_service<rm_interfaces::srv::SetMode>(
            "nahsor/set_mode", std::bind(&NahsorNode::setModeCallBack, this, _1, _2));
        // cam_client_ = std::make_shared<rm_cam::CamClient>(
        //     node_, camera_name, std::bind(&NahsorNode::process_image, this, _1, _2), false);
        wrapper_client_ = std::make_shared<rm_cam::WrapperClient>(
            node_, camera_name, imu_name, std::bind(&NahsorNode::process_image, this, _1, _2, _3));
        RCLCPP_INFO(node_->get_logger(), "Wrapper client create success.");
        sensor_msgs::msg::CameraInfo info;
        if (!wrapper_client_->get_camera_info(info))
        {
            RCLCPP_ERROR(node_->get_logger(), "get camera info failed!");
            return;
        }
        std::ostringstream oss;
        oss << "k:";
        for (auto &x : info.k)
        {
            oss << x << " ";
        }
        oss << ",d:";
        for (auto &x : info.d)
        {
            oss << x << " ";
        }
        RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());
        std::vector<double> camera_k(9, 0);
        std::copy_n(info.k.begin(), 9, camera_k.begin());
        mono_location_tool = std::make_shared<rm_util::MonoMeasureTool>(camera_k, info.d);

        nahsor_algo = std::make_shared<NahsorAlgo>(node_, mono_location_tool);
        if (auto_start)
        {
            current_mode_ = TaskMode::large;
            wrapper_client_->start();
            RCLCPP_INFO(node_->get_logger(), "auto start!");
        }
    }

    // void NahsorNode::process_image(cv::Mat &img, double time_stamp)
    // {
    //     if (current_mode_ == TaskMode::small)
    //     {
    //         int res = nahsor_algo->smallPred(img);
    //     }
    //     if (current_mode_ == TaskMode::large)
    //     {
    //         int res = nahsor_algo->bigPred(img, time_stamp);
    //     }
    // }
    void NahsorNode::process_image(std_msgs::msg::Header header, cv::Mat &img, geometry_msgs::msg::Pose pose)
    {
        //RCLCPP_INFO(node_->get_logger(), "%d", current_mode_);

        int res = -1;
        if (current_mode_ == TaskMode::small)
        {
            res = nahsor_algo->smallPred(img);
        }
        if (current_mode_ == TaskMode::large)
        {
            double time_stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9;
            res = nahsor_algo->bigPred(img, time_stamp_s);
        }
        if (current_mode_ == TaskMode::test)
        {
            res = nahsor_algo->noPred(img);
        }

        curr_pose_ = Eigen::Quaterniond(pose.orientation.w,
                                        pose.orientation.x,
                                        pose.orientation.y,
                                        pose.orientation.z);
        if (res == 0)
        {
            auto point = nahsor_algo->get_target();
            auto R_point = nahsor_algo->get_R_center();
            float pitch, yaw;
            mono_location_tool->calc_view_angle(point, pitch, yaw);
            auto angle = nahsor_algo->get_angle();
            //固定的距离和半径

            auto height = 0.8 * sin(angle) + 1.5;
            auto offset_pitch = std::make_shared<rm_trajectory::GetPitch>(30);
            auto target_pitch = offset_pitch->get_pitch(6, height, 30);
            auto target_yaw = atan2(0.8 * cos(angle), 6);
            pitch = rm_util::rad_to_deg(pitch);
            yaw = rm_util::rad_to_deg(yaw);

            auto euler_angles = rm_util::CoordinateTranslation::quat2euler(curr_pose_);
            float c_pitch, c_yaw;
            c_pitch = rm_util::rad_to_deg(euler_angles(1));
            c_yaw = -rm_util::rad_to_deg(euler_angles(0));
            // RCLCPP_INFO(node_->get_logger(), "target_pitch: %f, target_yaw :%f, yaw: %f", c_pitch - pitch + 3, c_yaw - yaw, yaw);
            if (this->current_mode_ != TaskMode::idle)
            {

                rm_interfaces::msg::GimbalCmd gimbal_cmd;
                gimbal_cmd.id = gimbal_cmd_id++;
                gimbal_cmd.position.pitch = c_pitch - pitch + 2.8;
                gimbal_cmd.position.yaw = c_yaw - yaw + 0.5;
                gimbal_cmd.velocity.pitch = 0;
                gimbal_cmd.velocity.yaw = 0;
                gimbal_cmd_pub_->publish(gimbal_cmd);
            }
            // auto q2photo = rm_util::CoordinateTranslation::euler2quat(Eigen::Vector3d(yaw * -1, 0, 0));
            // auto point_photo = rm_util::CoordinateTranslation::trans_quat_3d(point, q2photo);
            // auto q2cloud = Eigen::Quaterniond(pose.orientation.w,
            //                                   pose.orientation.x * -1,
            //                                   pose.orientation.y * -1,
            //                                   pose.orientation.z * -1);
            // auto point_cloud = rm_util::CoordinateTranslation::trans_quat_3d(point_photo, q2cloud);
        }
    }
    bool NahsorNode::setModeCallBack(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
    {
        //RCLCPP_INFO(node_->get_logger(), "%d", request->mode);
        response->success = true;
        if (request->mode == 0xbb)
        {
            current_mode_ = TaskMode::small;
            wrapper_client_->start();
        }
        else if (request->mode == 0xcc)
        {
            current_mode_ = TaskMode::large;
            wrapper_client_->start();
        }
        else if (request->mode == 0xdd)
        {
            current_mode_ = TaskMode::large;
            wrapper_client_->start();
        }
        else
        {
            current_mode_ = TaskMode::idle;
            wrapper_client_->stop();
        }
        return true;
    }

}