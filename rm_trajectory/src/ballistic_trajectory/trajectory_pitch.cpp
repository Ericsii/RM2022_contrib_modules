/*
 * @Author: your name
 * @Date: 2022-01-17 16:03:40
 * @LastEditTime: 2022-01-19 15:11:32
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scu_rm_ros/contrib_modules/rm_trajectory/src/ballistic_trajectory/trajectory_pitch.cpp
 */
#include "rm_trajectory/trajectory_pitch.hpp"
#include <math.h>
#include <iostream>

namespace rm_trajectory
{
    GetPitch::GetPitch(double initial_vel, double air_drag, double K, double correction)
    : initial_vel_(initial_vel), air_drag_(air_drag), K_(K), correction_(correction) {}

    double GetPitch::pitch_1(double target_h, double bullet_v) {

        double pitch = 0;

        if (bullet_v == 30) {
            pitch = 0.05985 * target_h + 2.794;           
        }

        if (bullet_v == 18) {
            pitch = 0.04518 * target_h + 3.257;
        }

        return pitch;
    } 
    double GetPitch::pitch_3(double target_h, double bullet_v) {

        double pitch = 0;

        if (bullet_v == 30){
            if (target_h > 0) {
                pitch = 0.01736 * target_h + 2.975;
                }
            else {
                pitch = 0.01736 * (target_h - 70) + 2.975;
            }            
        }

        if (bullet_v == 18) {
            pitch = 0.01759 * target_h + 4.546;
        }

        return pitch;
    }
    double GetPitch::pitch_5(double target_h, double bullet_v) {

        double pitch = 0;

        if (bullet_v == 30) {
            pitch = 0.01055 * target_h + 4.292;
        }

        if (bullet_v == 18) {
            pitch = 0.01059 * target_h + 6.482; 
        }

        return pitch;
    }
    double GetPitch::pitch_7(double target_h, double bullet_v) {

        double pitch = 0;

        if (bullet_v == 30) {
            pitch = 0.007561 * target_h + 5.16;
        }

        if (bullet_v == 18) {
            pitch = 0.007972 * target_h + 7.762;
        }

        return pitch;
    }

    double GetPitch::get_pitch(double target_distance, double target_h, double bullet_v) {

        bullet_v = bullet_v;

        target_distance = target_distance / 1000;

        double pitch;
        double f_1 = pitch_1(target_h, bullet_v);
        double f_3 = pitch_3(target_h, bullet_v);
        double f_5 = pitch_5(target_h, bullet_v);
        double f_7 = pitch_7(target_h, bullet_v);

        pitch = f_1 + (f_3 - f_1) / 2 * (target_distance - 1)
        + ((f_5 - f_3) / 2 - (f_3 - f_1) / 2) / 4 * (target_distance - 1) * (target_distance - 3)
        + (((f_7 - f_5) / 2 - (f_5 - f_3) / 2) / 4 - ((f_5 - f_3) / 2 - (f_3 - f_1) / 2) / 4) / 6 *
        (target_distance - 1) * (target_distance - 3) * (target_distance - 5);

        return pitch;

    }

    double GetPitch::get_pitch_model(double target_distance, double target_h, double bullet_v) {

        bullet_v = bullet_v;

        target_distance = target_distance / 1000;

        double pitch = 0, T_k = 0, T_nk = 0, v = 0, h_k = 0, error = 0;
        int while_times_in = 0, while_times_out = 0;

        while (1) {
            while (1) {
                v = bullet_v * cos(pitch * DEC) / (air_drag_ * bullet_v * cos(pitch * DEC) * T_k + 1);
                T_nk = T_k - (log(air_drag_ * bullet_v * cos(pitch * DEC) * T_k + 1) / air_drag_ - target_distance) / v;
                if (abs(T_nk - T_k) < 0.01) {
                    while_times_in = 0;
                    break;
                }
                T_k = T_nk;
                while_times_in += 1;

                if (while_times_in > 20) {
                    while_times_in = 0;
                    break;
                } 
            }
            T_k = abs(T_k);
            h_k = bullet_v * sin(pitch * DEC) * T_k - 4.9 * T_k * T_k;
            error = target_h / 1000 - h_k;

            if (abs(error) < 0.01) {
                while_times_out = 0;
                break;
            }

            if (while_times_out > 20) {
                while_times_out = 0;
                break;
            }
            pitch = pitch + K_ * error;
            while_times_out += 1;
        }

        return pitch + correction_;
    }

}
