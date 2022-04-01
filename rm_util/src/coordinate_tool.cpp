#include "rm_util/coordinate_tool.hpp"

#include <opencv2/core/eigen.hpp>

namespace rm_util
{
    Eigen::Vector3d CoordinateTranslation::trans_quat_3d(Eigen::Vector3d point3d, Eigen::Quaterniond q)
    {
        return q * point3d;
    }

    Eigen::Vector3d CoordinateTranslation::quat2euler(Eigen::Quaterniond q)
    {
        // 按照 ZYX 顺序转换(即 rpy)
        // return q.matrix().eulerAngles(2, 1, 0);

        // Eigen::Vector3d res;
        // auto q_copy = q.coeffs(); // [x y z w]

        // // roll (x-axis rotation)
        // double sinr_cosp = 2 * (q_copy[3] * q_copy[0] + q_copy[1] * q_copy[2]);
        // double cosr_cosp = 1 - 2 * (q_copy[0] * q_copy[0] + q_copy[1] * q_copy[1]);
        // res(0) = std::atan2(sinr_cosp, cosr_cosp);

        // // pitch (y-axis rotation)
        // double sinp = 2 * (q_copy[3] * q_copy[1] - q_copy[2] * q_copy[0]);
        // if (std::abs(sinp) >= 1)
        //     res(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        // else
        //     res(1) = std::asin(sinp);

        // // yaw (z-axis rotation)
        // double siny_cosp = 2 * (q_copy[3] * q_copy[2] + q_copy[0] * q_copy[1]);
        // double cosy_cosp = 1 - 2 * (q_copy[1] * q_copy[1] + q_copy[2] * q_copy[2]);
        // res(2) = std::atan2(siny_cosp, cosy_cosp);

        
        // RPY
        Eigen::Vector3d res;

        // roll
        res(0) = -std::atan2(2.0 * (q.x() * q.y() + q.z() * q.w()),
                            q.x() * q.x() - q.y() * q.y() - q.z() * q.z() + q.w() * q.w());

        // pitch
        res(1) = -std::asin(2.0 * (q.y() * q.w() - q.x() * q.z()));

        // yaw
        res(2) = std::atan2(2.0 * (q.y() * q.z() + q.x() * q.w()),
                            q.x() * q.x() + q.y() * q.y() - q.z() * q.z() - q.w() * q.w());

        return res;
    }

    Eigen::Quaterniond CoordinateTranslation::euler2quat(Eigen::Vector3d eulerAngle)
    {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
        return rollAngle * pitchAngle * yawAngle;
    }
} // namespace rm_util
