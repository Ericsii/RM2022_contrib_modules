# RM_FILTERS

rm_filters 卡尔曼滤波模块。

包含基础卡尔曼以及扩展卡尔曼滤波，以及匀加速和ctrv运动模型，为工具类函数。

## 环境

- Eigen3及以上 

## 运动模型(二维)

| 运动模型 |                状态量                | $Matrixx$ | $Matrixy$ |
| :------: | :----------------------------------: | :-------: | :-------: |
|   CTRV   | $x_k=[p_x, p_y, v, \theta, d\theta]$ |     5     |     2     |
|    CA    |      $x_k=[p_x, p_y, v_x, v_y]$      |     4     |     1     |

## 矩阵

* <font size=3>计算通用矩阵</font>

| 矩阵名称 |         说明         |          大小           |
| :------: | :------------------: | :---------------------: |
|  $x_k$   |       后验估计       |    $Matrixx\times1$     |
|  $x_l$   |  上一时刻的后验估计  |    $Matrixx\times1$     |
|  $x_p$   |       前验估计       |    $Matrixx\times1$     |
|   $B$    |  加速度状态转移矩阵  | $Matrixx\times Matrixy$ |
|   $K$    |      卡尔曼增益      | $Matrixx\times Matrixy$ |
|   $A$    |     状态转移矩阵     | $Matrixx\times Matrixx$ |
|   $H$    | 状态与测量量变化矩阵 | $Matrixy\times Matrixx$ |
|   $Q$    |    系统协方差矩阵    | $Matrixx\times Matrixx$ |
|   $P$    |     估计的协方差     | $Matrixx\times Matrixx$ |
|   $R$    |    测量噪声协方差    | $Matrixy\times Matrixy$ |
|   $U$    |      加速度矩阵      |    $Matrixy\times1$     |
|  $z_k$   |       观测矩阵       |    $Matrixy\times1$     |

- EKF计算额外用矩阵

| 矩阵名称 |             说明              |          大小           |
| :------: | :---------------------------: | :---------------------: |
|   $W$    |  EKF中运动模型的二阶偏导矩阵  | $Matrixx\times Matrixx$ |
|   $V$    | EKF中传感器模型的二阶偏导矩阵 | $Matrixy\times Matrixy$ |

## 滤波器使用

* KF滤波器的使用

```c++
Filters *kf_filter = new Kalman(x_p_k, x_l_k, A, H, Q, P, R, B, delta_t);
kf_filter->predict(U);
kf_filter->update(z_k);
```

- EKF滤波器的使用

```c++
//根据实际情况设定filter_interface.hpp里矩阵参数

//假如需要x,y,z,x_v,y_v三个参数，Matrix_x = 5
#define Matrix_x 5

//Matrix_y, 传感器状态向量参数个数，假如传感器有x,y,z三个参数，Matrix_y = 3
#define Matrix_y 3

//x_l_k = [x, y, z, x_v, y_v];当前车体状况
Eigen::MatrixXd x_l_k(Matrix_x, 1);
x_p_k = x_l_k;

//z_k = [x, y, z];三个方向上传感器观测值
Eigen::MatrixXd z_k(Matrix_y, 1);

//初步测试时，Q矩阵可以设为单位矩阵
Eigen::MatrixXd Q(Matrix_x, Matrix_x);//并初始化为单位矩阵

//不知道传感器误差情况下，可先初始化为相对单位矩阵（例如如果要进行滤波数据单位为mm，可初始化为1mm）
Eigen::MatrixXd R(Matrix_y, Matrix_y);

//加速度向量，即各个方向上的加速度，不加以控制时，可以初始化为0矩阵。
//如果设为0矩阵出现数值错误，注释掉filter_state.cpp里const_acc中Eigen::MatrixXd x_p = A * x_l_k + U * B;这条语句中的 U * B部分
Eigen::MatrixXd U(Matrix_x, Matrix_x);

//方差矩阵P,滤波器会自动更新，初始化为零矩阵即可
P = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);

Filters *ekf_filter = new ExKalman(x_p_k, x_l_k, Q, P, R, 
						MState::const_acc, MState::df_const_acc, MState::se_df_const_acc,
						MState::const_acc_sensor, MState::df_const_acc_sensor,
						MState::se_df_const_acc_sensor);
ekf_filter->predict(U, t);//t要参考帧率
ekf_filter->update(z_k);
```

## 开发日志

#### 不同运动轨迹下不同模型的滤波效果（与测量值相比）

##### 以下的测试均基于相同的Q，R，无加速度。

- 直线

| 模型 | measure_mse_x | predict_mse_x | measure_mse_y | predict_mse_y |
| ---- | ------------- | ------------- | ------------- | ------------- |
| CTRV | 108.2056      | 69.6243       | 95.1234       | 69.9184       |
| CA   | 108.2056      | 48.4138       | 95.1234       | 46.2228       |

- Z型

| 模型 | measure_mse_x | predict_mse_x | measure_mse_y | predict_mse_y |
| ---- | ------------- | ------------- | ------------- | ------------- |
| CTRV | 83.8717       | 58.5657       | 99.8821       | 41.8936       |
| CA   | 83.8717       | 66.6246       | 99.8821       | 68.0075       |

* 圆形

| 模型 | measure_mse_x | predict_mse_x | measure_mse_y | predict_mse_y |
| ---- | ------------- | ------------- | ------------- | ------------- |
| CTRV | 118.0300      | 40.5565       | 103.7283      | 62.7990       |
| CA   | 118.0300      | 82.0116       | 103.7283      | 73.4186       |

- 抛物线

| 模型 | measure_mse_x | predict_mse_x | measure_mse_y | predict_mse_y |
| ---- | ------------- | ------------- | ------------- | ------------- |
| CTRV | 103.9549      | 71.99         | 100.3734      | 100.0412      |
| CA   | 103.9549      | 76.2265       | 100.3734      | 79.3148       |

### （未解决）关于CTRV角速度的获取\更新

