#include "rm_sentry/sentry_node.hpp"
namespace rm_sentry
{
    SentryNode::SentryNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("Sentry", options);
        std::string camera_name = "mv_camera";
        std::string robot_color = "red";
        std::string imu_name = "imu/data_raw";
        std::string xml_path = "/home/scurm/Downloads/model-opt-int8.xml";
        std::string bin_path = "/home/scurm/Downloads/model-opt-int8.bin";
        double thre = 0.1;
        bool auto_start = false;
        bool debug = true;
        node_->declare_parameter("robot_color", "red");
        node_->declare_parameter("debug", debug);
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("auto_start", auto_start);
        node_->declare_parameter("imu_name", imu_name);
        node_->declare_parameter("xml_path", xml_path);

        node_->get_parameter("robot_color", robot_color);
        node_->get_parameter("debug", debug);
        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("auto_start", auto_start);
        node_->get_parameter("imu_name", imu_name);
        node_->get_parameter("xml_path", xml_path);

        bool is_red = (robot_color == "color");
        std::cout << xml_path << std::endl;
        using namespace std::placeholders;
        RCLCPP_INFO(node_->get_logger(), "Creating rcl pub&sub&client.");
        gimbal_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>("cmd_gimbal", 10);
        set_mode_srv_ = node_->create_service<rm_interfaces::srv::SetMode>(
            "auto_aim/set_mode", std::bind(&SentryNode::set_mode_cb, this, _1, _2));
        wrapper_client_ = std::make_shared<rm_cam::WrapperClient>(
            node_, camera_name, imu_name, std::bind(&SentryNode::process_fn, this, _1, _2, _3));
        RCLCPP_INFO(node_->get_logger(), "Create success");
        sensor_msgs::msg::CameraInfo cam_info;
        
        if (!wrapper_client_->get_camera_info(cam_info))
        {
            RCLCPP_FATAL(node_->get_logger(), "Get camera info failed");
            return;
        }
        std::ostringstream oss;
        oss << "k:";
        for (auto &x : cam_info.k)
        {
            oss << x << " ";
        }
        oss << ",d:";
        for (auto &x : cam_info.d)
        {
            oss << x << " ";
        }
        // RCLCPP_INFO(node_->get_logger(),"camera: %s, imu: %s", camera_name.c_str(), imu_name.c_str());
        RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());

        std::vector<double> camera_k(9, 0);
        std::copy_n(cam_info.k.begin(), 9, camera_k.begin());
        measure_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_k, cam_info.d);
        sentry_algo_ = std::make_shared<SentryAlgo>(node_, measure_tool_, is_red, xml_path, bin_path, thre);
        // RCLCPP_INFO(node_->get_logger(), "init finish");
        wrapper_client_->start();
    }

    void SentryNode::process_fn(std_msgs::msg::Header header,
                                cv::Mat &img,
                                geometry_msgs::msg::Quaternion q)
    {
        // RCLCPP_INFO(node_->get_logger(), "node process");

        double time_stamp_ms = header.stamp.sec * 1e3 + header.stamp.nanosec * 1e-6;
        curr_pose_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
        int res = sentry_algo_->process(time_stamp_ms, img, curr_pose_);
        rm_interfaces::msg::GimbalCmd gimbal_cmd;
        if (res == 0)
        {
            float offset_pitch, offset_yaw;
            offset_yaw = sentry_algo_->mTarget_yaw;
            offset_pitch = sentry_algo_->mTarget_pitch;
            gimbal_cmd.id = gimbal_cmd_id++;
            gimbal_cmd.type = 0x2a;
            gimbal_cmd.position.pitch = offset_pitch;
            gimbal_cmd.position.yaw = -offset_yaw;
            RCLCPP_INFO(node_->get_logger(),"pitch: %f, yaw: %f", offset_pitch, -offset_yaw);
            //gimbal_cmd_pub_->publish(gimbal_cmd);
        }
        else
        {
            gimbal_cmd.id = gimbal_cmd_id++;
            gimbal_cmd.type = 0x4a;
            gimbal_cmd.position.pitch = 0;
            gimbal_cmd.position.yaw = 0;
            gimbal_cmd_pub_->publish(gimbal_cmd);
        }
    }
    bool SentryNode::set_mode_cb(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                                 std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
    {
        (void)request;
        response->success = true;
        wrapper_client_->start();
        return true;
    }

}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_sentry::SentryNode)