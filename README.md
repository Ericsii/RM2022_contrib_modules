# contrib_modules

SCURM2022 附属ROS节点仓库

> 自瞄&能量机关&滤波器&弹道解算

## 介绍

此仓库包含：

<!-- TODO:ROS2 packages -->

## Pitch补偿

#### 使用方法

该包不支持ros节点单独运行，只能通过库依赖的方式调用

```c++
auto test_pitch = std::make_shared<rm_trajectory::GetPitch>(initial_vel);
pitch = test_pitch->get_pitch(target_distance, target_h, initial_vel);
```

#### 原始数据记录(待更新)

