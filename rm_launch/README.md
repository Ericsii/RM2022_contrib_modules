# RM_LAUNCH

`rm_launch`程序启动模块。

通过ros2 launch 启动所有节点，并通过component将多个节点组合在一起。
## 环境 
- ROS2-galactic, CMake 3.8, gcc 9.4.0, Ubuntu 20.04

## 启动
```shell
启动imu节点：ros2 launch rm_launch xxx_imu_up.py
启动其他功能节点：ros2 launch rm_launch xxx_all_up.py 
```

## 参数配置
通过 config/all_param.yaml 文件进行所有节点参数的配置
