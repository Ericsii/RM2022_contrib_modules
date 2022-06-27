clc
clear

pitch_1 = [-20, -18, -16, -14, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26];
h_1_laser = [585, 630, 680, 728, 773, 187, 230, 270, 310, 350, 400, 450, 490, 530, 570, 610, 645, 685, 710, 750, 795, 845, 895, 945];
h_1_robot = [997, 1002, 1010, 1017, 1022, 364, 369, 375, 383, 390, 400, 405, 410, 415, 420, 425, 430, 435, 440, 448, 455, 463, 470, 477];
h_1_real = [523, 569, 619, 670.5, 711.6, 130, 176, 217, 259, 297, 348.5, 396.5, 429, 476, 509.5, 548, 582, 636, 653, 689, 729, 778, 827, 879];
h_1 = h_1_real - h_1_robot;

pitch_3 = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30];
h_3_laser = [495, 615, 718, 838, 945, 400, 510, 620, 730, 840, 945, 1060, 1170, 1283, 1405, 1530, 1635, 1779, 1905, 2050, 2170];
h_3_robot = [1067, 1067, 1067, 1067, 1067, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400];
h_3_real = [326.7, 443, 558, 671, 787, 231.8, 349, 471.7, 574, 699.25, 787.5, 894.5, 1006.4, 1116.7, 1245, 1386, 1486, 1604.75, 1728, 1878, 1987];
h_3 = h_3_real - h_3_robot;

pitch_5 = [-14, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28];
h_5_robot = [2041, 2041, 2041, 1374, 1374, 1374, 1374, 1374, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400];
h_5_real = [314.75, 476, 646, 259, 414.5, 605, 820, 1029, 236, 370, 589, 773.5, 934, 1119.5, 1340.25, 1423, 1701.8, 1899.5, 2148.5, 2370.5, 2542, 2775];
h_5 = h_5_real - h_5_robot;

pitch_7 = [-8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24];
h_7_robot = [2041, 2041, 2041, 2041, 2041, 2041, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400];
h_7_real = [319, 571, 847.2, 1103.75, 1424, 1587, 250, 510, 753.75, 1037, 1223.5, 1541, 1819, 2053, 2308.7, 2705.7, 2977];
h_7 = h_7_real - h_7_robot;

v_init = 30;
T_k = 0; T_nk = 0; k_1 = 0.022;
theta =0; T = 0; v = 0;
h_k = 0; h_r = 0; error = 0;
target_distance = 1; target_v = 0; K = 2;
get_pitch_1 = []; pitch_error_1 = [];
correction = 2.5;
for i = 1:24 
    h_r = h_1(i) / 1000;
    while (1)
        while (1)
            v = v_init * cosd(theta) / (k_1 * v_init * cosd(theta) * T_k + 1);
            T_nk = T_k - (log(k_1 * v_init * cosd(theta) * T_k + 1) / k_1 - target_distance) / v;
            if abs(T_nk - T_k) < 0.01 
                break;
            end
            T_k = T_nk;
        end
        T = abs(T_k);
        h_k = v_init * sind(theta) * T - 4.9 * T * T;
        error = h_r - h_k;
        if abs(error) < 0.01 
            break;
        end
        theta = theta + K * error;
    end
    get_pitch_1(i) = theta + correction;
end
subplot(2,2,1)
plot(h_1,pitch_1)
hold on
plot(h_1,get_pitch_1)
legend("test", "model")
title("1米模型与测试结果对比")
pitch_error_1 = pitch_1 - get_pitch_1;
mean(pitch_error_1)

target_distance = 3;
get_pitch_3 = [];
pitch_error_3 = [];
for i = 1:21 
    h_r = h_3(i) / 1000;
    while (1)
        while (1)
            v = v_init * cosd(theta) / (k_1 * v_init * cosd(theta) * T_k + 1);
            T_nk = T_k - (log(k_1 * v_init * cosd(theta) * T_k + 1) / k_1 - target_distance) / v;
            if abs(T_nk - T_k) < 0.01 
                break;
            end
            T_k = T_nk;
        end
        T = abs(T_k);
        h_k = v_init * sind(theta) * T - 4.9 * T * T;
        error = h_r - h_k;
        if abs(error) < 0.01 
            break;
        end
        theta = theta + K * error;
    end
    get_pitch_3(i) = theta + correction;
end
subplot(2,2,2)
plot(h_3,pitch_3)
hold on
plot(h_3,get_pitch_3)
legend("test", "model")
title("3米模型与测试结果对比")
pitch_error_3 = pitch_3 - get_pitch_3;
mean(pitch_error_3)

target_distance = 5;
get_pitch_5 = [];
pitch_error_5 = [];
for i = 1:22 
    h_r = h_5(i) / 1000;
    while (1)
        while (1)
            v = v_init * cosd(theta) / (k_1 * v_init * cosd(theta) * T_k + 1);
            T_nk = T_k - (log(k_1 * v_init * cosd(theta) * T_k + 1) / k_1 - target_distance) / v;
            if abs(T_nk - T_k) < 0.01 
                break;
            end
            T_k = T_nk;
        end
        T = abs(T_k);
        h_k = v_init * sind(theta) * T - 4.9 * T * T;
        error = h_r - h_k;
        if abs(error) < 0.01 
            break;
        end
        theta = theta + K * error;
    end
    get_pitch_5(i) = theta + correction;
end
subplot(2,2,3)
plot(h_5,pitch_5)
hold on
plot(h_5,get_pitch_5)
legend("test", "model")
title("5米模型与测试结果对比")
pitch_error_5 = pitch_5 - get_pitch_5;
mean(pitch_error_5)

target_distance = 7;
get_pitch_7 = [];
pitch_error_7 = [];
for i = 1:17
    h_r = h_7(i) / 1000;
    while (1)
        while (1)
            v = v_init * cosd(theta) / (k_1 * v_init * cosd(theta) * T_k + 1);
            T_nk = T_k - (log(k_1 * v_init * cosd(theta) * T_k + 1) / k_1 - target_distance) / v;
            if abs(T_nk - T_k) < 0.01 
                break;
            end
            T_k = T_nk;
        end
        T = abs(T_k);
        h_k = v_init * sind(theta) * T - 4.9 * T * T;
        error = h_r - h_k;
        if abs(error) < 0.01 
            break;
        end
        theta = theta + K * error;
    end
    get_pitch_7(i) = theta + correction;
end
subplot(2,2,4)
plot(h_7,pitch_7)
hold on
plot(h_7,get_pitch_7)
legend("test", "model")
title("7米模型与测试结果对比")
pitch_error_7 = pitch_7 - get_pitch_7;
mean(pitch_error_7)