#include "rm_util/measure_tool.hpp"

namespace rm_util
{
    MonoMeasureTool::MonoMeasureTool(
        std::vector<double> camera_intrinsic,
        std::vector<double> camera_distortion)
    {
        if (camera_intrinsic.size() != 9)
        {
            throw std::invalid_argument("the size of camera intrinsic must be 9 (equal 3*3)");
        }

        cv::Mat camera_intrinsic_mat(camera_intrinsic, true);
        camera_intrinsic_mat = camera_intrinsic_mat.reshape(0, 3);
        camera_intrinsic_ = camera_intrinsic_mat.clone();

        cv::Mat camera_distortion_mat(camera_distortion, true);
        camera_distortion_mat = camera_distortion_mat.reshape(0, 1);
        camera_distortion_ = camera_distortion_mat.clone();
    }

    bool MonoMeasureTool::set_camera_info(
        std::vector<double> camera_intrinsic,
        std::vector<double> camera_distortion)
    {
        if (camera_intrinsic.size() != 9)
        {
            // the size of camera intrinsic must be 9 (equal 3*3)
            return false;
        }

        cv::Mat camera_intrinsic_mat(camera_intrinsic, true);
        camera_intrinsic_mat = camera_intrinsic_mat.reshape(0, 3);
        camera_intrinsic_ = camera_intrinsic_mat.clone();

        cv::Mat camera_distortion_mat(camera_distortion, true);
        camera_distortion_mat = camera_distortion_mat.reshape(0, 1);
        camera_distortion_ = camera_distortion_mat.clone();

        return true;
    }

    bool MonoMeasureTool::solve_pnp(
        std::vector<cv::Point2f> &points2d,
        std::vector<cv::Point3f> &points3d,
        cv::Point3f &position,
        cv::Mat &rotation)
    {
        if (points2d.size() != points3d.size())
        {
            throw std::invalid_argument("Points size does not match!");
            return false;
        }

        cv::Mat trans = cv::Mat::zeros(3, 1, CV_64FC1); // 平移变换向量
        cv::Mat r;                                      // 旋转向量
        cv::solvePnP(points3d, points2d,
                     camera_intrinsic_, camera_distortion_,
                     r, trans,
                     false, cv::SOLVEPNP_EPNP);

        position = cv::Point3f(trans);
        rotation = r.clone();
        return true;
    }

    // refer to :http://www.cnblogs.com/singlex/p/pose_estimation_1_1.html
    // 根据输入的参数将图像坐标转换到相机坐标中
    // 输入为图像上的点坐标
    // double distance 物距
    // 输出3d点坐标的单位与distance（物距）的单位保持一致
    cv::Point3f MonoMeasureTool::unproject(cv::Point2f p, double distance)
    {
        auto fx = camera_intrinsic_.ptr<double>(0)[0];
        auto u0 = camera_intrinsic_.ptr<double>(0)[2];
        auto fy = camera_intrinsic_.ptr<double>(1)[1];
        auto v0 = camera_intrinsic_.ptr<double>(1)[2];

        double zc = distance;
        double xc = (p.x - u0) * distance / fx;
        double yc = (p.y - v0) * distance / fy;
        return cv::Point3f(xc, yc, zc);
    }

    void MonoMeasureTool::calc_view_angle(cv::Point2f p, float &pitch, float &yaw)
    {
        auto fx = camera_intrinsic_.ptr<double>(0)[0];
        auto u0 = camera_intrinsic_.ptr<double>(0)[2];
        auto fy = camera_intrinsic_.ptr<double>(1)[1];
        auto v0 = camera_intrinsic_.ptr<double>(1)[2];

        pitch = atan2((p.y - v0), fy);
        yaw = atan2((p.x - u0), fx);
    }

} // namespace rm_util
