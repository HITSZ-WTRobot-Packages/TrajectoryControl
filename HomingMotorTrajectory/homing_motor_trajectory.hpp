/**
 * @file    homing_motor_trajectory.hpp
 * @author  syhanjin
 * @date    2026-04-17
 */
#pragma once

#include "motor_trajectory.hpp"
#include "main.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace trajectory
{

struct MotorCalibration
{
    enum class State
    {
        Idle,
        Finding, ///< 检测堵转位置
        Done,
    };

    State state = State::Idle;

    float max_current_on_working = 0;

    bool     stalling         = false;
    uint32_t stall_start_tick = 0;
    float    stall_angle      = 0.0f;
};

template <size_t MotorNum> class HomingMotorTrajectory : public MotorTrajectory<MotorNum>
{
    static_assert(MotorNum > 0, "HomingMotorTrajectory requires at least one motor");

    using Parent = MotorTrajectory<MotorNum>;

public:
    using ProfileConfig = typename Parent::ProfileConfig;

    struct CalibrationConfig
    {
        float    speed;       ///< 堵转寻点速度，单位跟 MotorVelController::setRef 保持一致
        float    max_current; ///< 堵转寻点时速度环输出上限
        uint32_t min_ticks;   ///< 堵转最小持续时间，单位 ms

        float offset; ///< 零点相对于堵转位置的偏移

        float dead_angle = 0.1f; ///< 堵转检测过程允许的角度误差，单位 deg
    };

    HomingMotorTrajectory(MotorTrajectory<MotorNum>&& trajectory,
                          const CalibrationConfig&    calibration_config) :
        MotorTrajectory<MotorNum>(std::move(trajectory)), cfg_(calibration_config)
    {
    }

    [[nodiscard]] bool enabled() const { return Parent::enabled() && isCalibrated(); }

    void startCalibration()
    {
        Parent::disable();

        // 对每个电机单独启动校准
        for (size_t i = 0; i < MotorNum; i++)
        {
            auto& ctrl  = Parent::ctrl_[i];
            auto& calib = motor_calib_[i];

            calib.max_current_on_working = ctrl->getPID().getConfig().abs_output_max;

            ctrl->getPID().setOutputMax(cfg_.max_current);
            ctrl->setRef(cfg_.speed);
            ctrl->enable();

            calib.stall_start_tick = 0;
            calib.stalling      = false;
            calib.state         = MotorCalibration::State::Finding;
        }

        state_ = CalibrationState::MotorFinding;
    }

    void profileUpdate(const float dt) { Parent::profileUpdate(dt); }

    void errorUpdate() { Parent::errorUpdate(); }

    void controllerUpdate()
    {
        if (state_ == CalibrationState::MotorFinding)
        {
            bool all_done = true;

            for (size_t i = 0; i < MotorNum; i++)
            {
                auto& ctrl = Parent::ctrl_[i];
                ctrl->update();

                auto& calib = motor_calib_[i];
                if (calib.state == MotorCalibration::State::Done)
                {
                    continue;
                }
                all_done = false;

                if (std::abs(ctrl->getPID().getOutput()) <= 0.99f * cfg_.max_current)
                {
                    calib.stalling = false;
                }
                else
                {
                    const uint32_t now = HAL_GetTick();
                    if (!calib.stalling)
                    {
                        calib.stalling         = true;
                        calib.stall_start_tick = now;
                        calib.stall_angle      = ctrl->getMotor()->getAngle();
                    }
                    else if (now - calib.stall_start_tick >= cfg_.min_ticks)
                    {
                        if (std::abs(ctrl->getMotor()->getAngle() - calib.stall_angle) <
                            cfg_.dead_angle)
                        {
                            calib.state = MotorCalibration::State::Done;
                            ctrl->getMotor()->resetAngle();
                            ctrl->setRef(0.0f);
                            ctrl->getPID().setOutputMax(calib.max_current_on_working);
                            // 校准完成通过速度环锁定
                        }
                        else
                        {
                            calib.stall_start_tick = now;
                            calib.stall_angle   = ctrl->getMotor()->getAngle();
                        }
                    }
                }
            }

            if (all_done)
            {
                state_ = CalibrationState::Zeroing;
                Parent::enable();
                setTarget(0);
            }
            return;
        }
        if (state_ == CalibrationState::Zeroing)
        {
            if (Parent::isFinished())
                state_ = CalibrationState::Done;
        }
        Parent::controllerUpdate();
    }

    bool setTarget(const float target, const LinkMode link_mode, const ProfileConfig& config)
    {
        return Parent::setTarget(cfg_.offset + target, link_mode, config);
    }
    bool setTarget(const float target) { return Parent::setTarget(cfg_.offset + target); }
    bool setTarget(const float target, const LinkMode link_mode)
    {
        return Parent::setTarget(cfg_.offset + target, link_mode);
    }
    bool setTarget(const float target, const ProfileConfig& config)
    {
        return Parent::setTarget(cfg_.offset + target, config);
    }

    bool isCalibrated() const { return state_ == CalibrationState::Done; }

private:
    enum class CalibrationState
    {
        Idle,
        MotorFinding,
        Zeroing,
        Done,
    };

    CalibrationConfig cfg_{};

    CalibrationState state_ = CalibrationState::Idle;

    std::array<MotorCalibration, MotorNum> motor_calib_{};
};

} // namespace trajectory
