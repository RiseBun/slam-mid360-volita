//
// Created by xiang on 2021/11/11.
//

#include "static_imu_init.h"
#include "common/math_utils.h"

#include <cmath>
#include <glog/logging.h>

namespace zjloc
{

    bool StaticIMUInit::AddIMU(const IMU &imu)
    {
        if (init_success_)
        {
            return true;
        }

        if (options_.use_speed_for_static_checking_ && !is_static_)
        {
            LOG(WARNING) << "等待车辆静止";
            init_imu_deque_.clear();
            return false;
        }

        if (init_imu_deque_.empty())
        {
            // 记录初始静止时间
            init_start_time_ = imu.timestamp_;
        }

        // 记入初始化队列
        init_imu_deque_.push_back(imu);

        while (init_imu_deque_.size() > options_.init_imu_queue_max_size_)
        {
            init_imu_deque_.pop_front();
        }

        current_time_ = imu.timestamp_;

        double init_time = imu.timestamp_ - init_start_time_; // 初始化经过时间
        if (init_time >= options_.init_time_seconds_)
        {
            TryInit();

            // Keep a sliding time window so startup motion ages out instead of
            // contaminating every later initialization attempt.
            while (init_imu_deque_.size() > 10 &&
                   imu.timestamp_ - init_imu_deque_.front().timestamp_ > options_.init_time_seconds_)
            {
                init_imu_deque_.pop_front();
            }
            if (!init_imu_deque_.empty())
            {
                init_start_time_ = init_imu_deque_.front().timestamp_;
            }
        }

        return init_success_;
    }

    bool StaticIMUInit::TryInit()
    {
        if (init_imu_deque_.size() < 10)
        {
            return false;
        }

        // 计算均值和方差
        Vec3d mean_gyro, mean_acce;
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU &imu)
                                    { return imu.gyro_; });
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU &imu)
                                    { return imu.acce_; });

        // 以acce均值为方向，取9.8长度为重力
        LOG(INFO) << "mean acce: " << mean_acce.transpose();
        if (!mean_acce.allFinite() || mean_acce.norm() < 1e-6)
        {
            LOG(ERROR) << "invalid accelerometer mean during IMU initialization";
            return false;
        }
        const Vec3d raw_mean_acce = mean_acce;
        gravity_ = -raw_mean_acce / raw_mean_acce.norm() * options_.gravity_norm_;

        // 重新计算加计的协方差
        Vec3d corrected_mean_acce;
        math::ComputeMeanAndCovDiag(init_imu_deque_, corrected_mean_acce, cov_acce_,
                                    [this](const IMU &imu)
                                    { return imu.acce_ + gravity_; });

        // 检查IMU噪声
        if (cov_gyro_.norm() > options_.max_static_gyro_var)
        {
            LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
            return false;
        }

        if (cov_acce_.norm() > options_.max_static_acce_var)
        {
            LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var;
            return false;
        }

        if (mean_gyro.norm() > options_.max_static_gyro_mean_)
        {
            LOG(ERROR) << "IMU is rotating during initialization: " << mean_gyro.norm()
                       << " > " << options_.max_static_gyro_mean_;
            return false;
        }

        if (std::abs(raw_mean_acce.norm() - options_.gravity_norm_) > options_.max_gravity_norm_error_)
        {
            LOG(ERROR) << "accelerometer norm is not consistent with a stationary sensor: "
                       << raw_mean_acce.norm() << " vs " << options_.gravity_norm_;
            return false;
        }

        // 估计测量噪声和零偏
        init_bg_ = mean_gyro;
        // At rest: 0 = R * (mean_acce - ba) + gravity. gravity_ is
        // expressed in the initial IMU frame here, so ba = mean + gravity.
        init_ba_ = raw_mean_acce + gravity_;

        LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
                  << ", ba = " << init_ba_.transpose();
        LOG(INFO) << "gyro sq = " << cov_gyro_.transpose()
                  << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
                  << ", norm: " << gravity_.norm();
        LOG(INFO) << "mean gyro: " << mean_gyro.transpose()
                  << " acce: " << raw_mean_acce.transpose();
        mean_acc_ = raw_mean_acce;
        init_success_ = true;
        return true;
    }

} // namespace zjloc
