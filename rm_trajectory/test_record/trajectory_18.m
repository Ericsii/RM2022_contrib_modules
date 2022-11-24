clc
clear 

pitch_1 = [-20 -18 -16 -14 -12 -10 -8 -6 -4 -2 0 2 4 6 8 10 12 14 16 18 20 22 24];
h_1_robot = [1060 1060 1060 1060 1060 1060 1060 1060 1060 1060 390 390 390 390 390 390 390 390 390 390 390 390 390];
h_1_real = [522 582 650 690 720 770 816 867 900 935 307 365 408 452 500 541 600 622 660 708 764 805 846];
h_1 = h_1_real - h_1_robot;

pitch_3 = [-10 -8 -6 -4 -2 0 2 4 6 8 10 12 14 16 18 20 22];
h_3_robot = [1060 1060 1060 1060 1060 390 390 390 390 390 390 390 390 390 390 390 390];
h_3_real = [221 359 460 548 685 152 257 377 475 573 679 825 951 1025 1164 1260 1374];
h_3 = h_3_real - h_3_robot;

pitch_5 = [-6 -4 -2 0 2 4 6 8 10 12 14 16 18 20];
h_5_robot = [1364 1364 1364 1364 1364 1364 390 390 390 390 390 390 390 390];
h_5_real = [186 377 581 813 939 1156 283 533 675 842 1086 1296 1526 1696];
h_5 = h_5_real - h_5_robot;

pitch_7 = [2 4 6 8 10 12 14 16 18 20 22 24 26];
h_7_robot = [1060 1060 390 390 390 390 390 390 390 390 390 390 390];
h_7_real = [299 563 169 444 705 958 1172 1424 1671 1964 2129 2452 2632];
h_7 = h_7_real - h_7_robot;

v_init = 18;
T_k = 0; T_nk = 0; k_1 = 0.03;
theta =0; T = 0; v = 0;
h_k = 0; h_r = 0; error = 0;
target_distance = 1; target_v = 0; K = 2;
get_pitch_1 = []; pitch_error_1 = [];
correction = 2;
for i = 1:23 
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
for i = 1:17 
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
for i = 1:14 
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
for i = 1:13
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