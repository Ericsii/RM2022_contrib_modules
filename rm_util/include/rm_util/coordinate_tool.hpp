#ifndef RM_UTIL__COORDINATE_TOOL_HPP
#define RM_UTIL__COORDINATE_TOOL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rm_util
{
    // 坐标转换类
    class CoordinateTranslation
    {
    public:
        /**
         * @brief 使用四元数进行3维坐标变换
         * 
         * @param point3d 3维坐标点
         * @param q 四元数
         * @return Eigen::Vector3d 变换后3维点坐标
         */
        static Eigen::Vector3d trans_quat_3d(Eigen::Vector3d point3d, Eigen::Quaterniond q);

        /**
         * @brief 四元数转换为欧拉角
         * 
         * @param q 四元数
         * @return Eigen::Vector3d 欧拉角(RPY)
         */
        static Eigen::Vector3d quat2euler(Eigen::Quaterniond q);
    };
} // namespace rm_util

#endif // RM_UTIL__COORDINATE_TOOL_HPP