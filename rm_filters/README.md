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
Eigen::MatrixXd Q(rm_filters::Matrix_x, rm_filters::Matrix_x);
Q = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);		//需要调参(本定义方式为全0矩阵)

Eigen::MatrixXd R(rm_filters::Matrix_y, rm_filters::Matrix_y);
R = Eigen::MatrixXd::Zero(Matrix_y, Matrix_y);		//需要调参(本定义方式为全0矩阵)

Filters *kf_filter = new Kalman(Q, R);

Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
z_k(0, 0) = position3d_world(0, 0), z_k(1, 0) = position3d_world(1, 0);
z_k(2, 0) = position3d_world(2, 0);	//传入当前装甲板信息

kf_filter->init(z_k);				//初始化滤波器

Eigen::MatrixXd U(rm_filters::Matrix_x, rm_filters::Matrix_x);
U = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);		//暂时不需要调参

kf_filter->predict(U, t);
Eigen::MatrixXd x_k = kf_filter->update(z_k);		//更新，此时的x_k即为滤波后状态向量
```

- EKF滤波器的使用

```c++
Eigen::MatrixXd Q(rm_filters::Matrix_x, rm_filters::Matrix_x);
Q = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);		//需要调参(本定义方式为全0矩阵)

Eigen::MatrixXd R(rm_filters::Matrix_y, rm_filters::Matrix_y);
R = Eigen::MatrixXd::Zero(Matrix_y, Matrix_y);		//需要调参(本定义方式为全0矩阵)


Filters *ekf_filter = new ExKalman(Q, R, 
						rm_filters::MState::const_acc, 
                        rm_filters::MState::df_const_acc, 						 	                               rm_filters::MState::se_df_const_acc, 					                                 rm_filters::MState::const_acc_sensor, 
                        rm_filters::MState::df_const_acc_sensor,
						rm_filters::MState::se_df_const_acc_sensor);

Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
z_k(0, 0) = position3d_world(0, 0), z_k(1, 0) = position3d_world(1, 0);
z_k(2, 0) = position3d_world(2, 0);	//传入当前装甲板信息

ekf_filter->init(z_k);		//初始化滤波器
Eigen::MatrixXd U(rm_filters::Matrix_x, rm_filters::Matrix_x);
U = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);		//暂时不需要调参

ekf_filter->predict(U, t);	//预测
Eigen::MatrixXd x_k = ekf_filter->update(z_k);	//更新，此时的x_k即为滤波后状态向量
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

---

## 参数记录

### 28/3 步兵参数

```c++
	Q(0, 0) = 0.01; Q(0, 1) = 0;  Q(0, 2) = 0; 
	Q(0, 3) = 0; Q(0, 4) = 0;  Q(0, 5) = 0;
	Q(1, 0) = 0; Q(1, 1) = 0.01;  Q(1, 2) = 0; 
	Q(1, 3) = 0; Q(1, 4) = 0;  Q(1, 5) = 0;
	Q(2, 0) = 0; Q(2, 1) = 0;  Q(2, 2) = 0.005; 
	Q(2, 3) = 0; Q(2, 4) = 0;  Q(2, 5) = 0;
	Q(3, 0) = 0; Q(3, 1) = 0;  Q(3, 2) = 0; 
	Q(3, 3) = 0.01; Q(3, 4) = 0;  Q(3, 5) = 0;
    Q(4, 0) = 0; Q(4, 1) = 0;  Q(4, 2) = 0; 
	Q(4, 3) = 0; Q(4, 4) = 0.01;  Q(4, 5) = 0;
	Q(5, 0) = 0; Q(5, 1) = 0;  Q(5, 2) = 0;
	Q(5, 3) = 0; Q(5, 4) = 0;  Q(5, 5) = 0.005;
	
    R(0, 0) = 0.005, R(0, 1) = 0, R(0, 2) = 0;
    R(1, 0) = 0, R(1, 1) = 0.005, R(1, 2) = 0;
    R(2, 0) = 0, R(2, 1) = 0, R(2, 2) = 0.0025;
```

