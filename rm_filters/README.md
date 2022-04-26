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
|  $x_k$   |       后验估计       |    $Matrixx\times 1$     |
|  $x_l$   |  上一时刻的后验估计  |    $Matrixx\times 1$     |
|  $x_p$   |       前验估计       |    $Matrixx\times 1$     |
|   $B$    |  加速度状态转移矩阵  | $Matrixx\times Matrixy$ |
|   $K$    |      卡尔曼增益      | $Matrixx\times Matrixy$ |
|   $A$    |     状态转移矩阵     | $Matrixx\times Matrixx$ |
|   $H$    | 状态与测量量变化矩阵 | $Matrixy\times Matrixx$ |
|   $Q$    |    系统协方差矩阵    | $Matrixx\times Matrixx$ |
|   $P$    |     估计的协方差     | $Matrixx\times Matrixx$ |
|   $R$    |    测量噪声协方差    | $Matrixy\times Matrixy$ |
|   $u$    |     加速度控制向量   |    $Matrixy\times1$     |
|  $z_k$   |       观测矩阵       |    $Matrixy\times1$     |

- EKF计算额外用矩阵

| 矩阵名称 |             说明              |          大小           |
| :------: | :---------------------------: | :---------------------: |
|   $W$    |  EKF中运动模型的二阶偏导矩阵  | $Matrixx\times Matrixx$ |
|   $V$    | EKF中传感器模型的二阶偏导矩阵 | $Matrixy\times Matrixy$ |

## 滤波器使用

* KF滤波器的使用

```c++
    using namespace rm_filters;
    std::shared_ptr<FilterInterface> filter; // 抽象类指针
    auto kf = std::make_shared<KalmanFilter>(6, 3);	// 构造滤波器

    Eigen::MatrixXd A(6, 6); // 状态转移矩阵
    A << 1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd B(6, 6); // 控制矩阵
    B << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd H(3, 6); // 测量矩阵
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;

    Eigen::MatrixXd Q(6, 6); // 过程协方差
    Q << 0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01;

    Eigen::MatrixXd R(3, 3); // 观测协方差
    R << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;

    kf->A = A;
    kf->B = B;
    kf->H = H;
    kf->Q = Q;
    kf->R = R;
    filter = kf;
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6); // 初始状态

    filter->init(x_0); // 初始化(过程协方差矩阵置为0)

    for(int i = 0; i < 100; ++i)
    {
        Eigen::VectorXd u(6);
        Eigen::VectorXd z(3);
        u << 0, 0, 0, 0, 0, 0;
        z << i, i, i;
        filter->predict(u); // 预测
        auto x = filter->update(z); // 观测更新
    }
```

- EKF滤波器的使用

```c++
    using namespace rm_filters;
    std::shared_ptr<FilterInterface> filter; // 抽象类指针

    Eigen::MatrixXd Q(6, 6); // 过程协方差
    Q << 0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01;

    Eigen::MatrixXd R(3, 3); // 测量协方差
    R << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;

    auto ekf = std::make_shared<ExKalmanFilter>(6, 3, 0, Q, R); // 构造扩展卡尔曼滤波器
	
	// 使用恒定加速度运动模型
    ekf->base_state = MState::const_acc;
    ekf->df_state = MState::df_const_acc;
    ekf->se_df_state = MState::se_df_const_acc;
    ekf->base_sensor = MState::const_acc_sensor;
    ekf->df_sensor = MState::df_const_acc_sensor;
    ekf->se_df_sensor = MState::se_df_const_acc_sensor;

    filter = ekf;

    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6); // 初始状态
    filter->init(x_0);

    for (int i = 0; i < 100; ++i)
    {
        Eigen::VectorXd u(6);
        Eigen::VectorXd z(3);
        u << 0, 0, 0, 0, 0, 0;
        z << i, i, i;
        filter->predict(u, 0.01); // 预测 u, delta_t
        auto x = filter->update(z); // 更新
    }
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

