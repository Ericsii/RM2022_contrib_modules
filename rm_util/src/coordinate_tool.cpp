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
        return q.matrix().eulerAngles(0, 1, 2);
    }
} // namespace rm_util
