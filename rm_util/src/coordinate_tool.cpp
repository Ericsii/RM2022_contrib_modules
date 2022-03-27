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
        // return q.matrix().eulerAngles(0, 1, 2);

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
        double q0 = q.w();
        double q1 = q.x();
        double q2 = q.y();
        double q3 = q.z();
        
        // /* yaw  -pi/2----pi/2 */
        // res(0) = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
        // //imu.yaw = -atan2(2*(q1*q2 + q0*q3), q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
        // /* pitch  -pi/2----pi/2 */
        // res(1) = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        // /* roll   -pi----pi  */
        // res(2) = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;

        return res;
    }
} // namespace rm_util
