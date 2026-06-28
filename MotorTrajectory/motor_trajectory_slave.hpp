/**
 * @file    motor_trajectory_slave.hpp
 * @author  syhanjin
 * @date    2026-06-08
 */
#pragma once

#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "RingBuffer.hpp"
#include "isr_lock.h"

#include <cstddef>
#include <type_traits>

namespace trajectory
{

template <size_t MotorNum, size_t BufferCapacity = 0, bool Offline = false>
class MotorTrajectorySlave
{
    static_assert(MotorNum > 0, "MotorTrajectorySlave requires at least one motor");
    static_assert(Offline || BufferCapacity >= 2,
                  "MotorTrajectorySlave buffer capacity must be >= 2 when Offline is false");

public:
    struct TargetPoint
    {
        float q;  ///< 目标输出角度，单位 deg。
        float dq; ///< 目标输出角度导数，单位 deg/s。
    };

    MotorTrajectorySlave(controllers::MotorVelController* motor_controllers[MotorNum],
                         const PD::Config&                error_pd_cfg)
    {
        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i] = motor_controllers[i];

        for (auto& p : pd_)
            p.setConfig(error_pd_cfg);
    }

    MotorTrajectorySlave(controllers::MotorVelController (&motors)[MotorNum],
                         const PD::Config& error_pd_cfg)
    {
        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i] = &motors[i];

        for (auto& p : pd_)
            p.setConfig(error_pd_cfg);
    }

    template <size_t N = MotorNum, typename = std::enable_if_t<N == 1>>
    MotorTrajectorySlave(controllers::MotorVelController* motor_controller,
                         const PD::Config&                error_pd_cfg) :
        ctrl_{ motor_controller }, pd_{ PD(error_pd_cfg) }
    {
    }

    template <bool B = !Offline, typename = std::enable_if_t<B>>
    bool pushTrajectoryPoint(const TargetPoint& point)
    {
        ISRGuard lock{};
        return cmd_buffer_.push(point);
    }

    /**
     * 从缓冲区取出一个外部轨迹点作为当前目标。
     */
    template <bool B = !Offline, typename = std::enable_if_t<B>> void profileUpdate()
    {
        if (!enabled() || locked())
            return;

        TargetPoint point{};
        if (cmd_buffer_.pop(point))
        {
            updateTarget(point);
            stopped_ = false;
        }

        if (!stopped_)
            applyTargetWithLastError();
    }

    template <bool B = !Offline, typename = std::enable_if_t<B>> void profileUpdate(const float dt)
    {
        (void)dt;
        profileUpdate();
    }

    template <bool B = Offline, typename = std::enable_if_t<B>> void profileUpdate(const TargetPoint& point)
    {
        updateTarget(point);
        stopped_ = false;
        applyTargetWithLastError();
    }

    void errorUpdate()
    {
        if (!enabled() || locked())
            return;

        for (size_t i = 0; i < MotorNum; ++i)
        {
            const float error_output = pd_[i].calc(target_.q, ctrl_[i]->getMotor()->getAngle());
            ctrl_[i]->setRef(dps2rpm(target_.dq + error_output));
        }
    }

    void controllerUpdate()
    {
        if (!enabled() || locked())
            return;

        for (auto& ctrl : ctrl_)
            ctrl->update();
    }

    void stop()
    {
        {
            ISRGuard lock{};
            stopped_   = true;
            target_.q  = getCurrentAvePosition();
            target_.dq = 0.0f;
            if constexpr (!Offline)
                cmd_buffer_.clear();
        }

        for (auto& p : pd_)
            p.reset();

        if (enabled())
            applyTargetWithLastError();
    }

    bool enable()
    {
        bool success = true;
        for (auto& ctrl : ctrl_)
            success &= ctrl->enable();

        if (!success)
        {
            for (auto& ctrl : ctrl_)
                ctrl->disable();
            {
                ISRGuard lock{};
                enabled_ = false;
            }
            return false;
        }

        {
            ISRGuard lock{};
            enabled_ = true;
        }
        stop();
        return true;
    }

    void disable()
    {
        if (enabled_)
            stop();

        for (auto& ctrl : ctrl_)
            ctrl->disable();

        {
            ISRGuard lock{};
            enabled_ = false;
        }
    }

    [[nodiscard]] bool enabled() const { return enabled_; }

    void               lock()
    {
        ISRGuard guard{};
        lock_ = true;
    }
    void               unlock()
    {
        ISRGuard guard{};
        lock_ = false;
    }
    [[nodiscard]] bool locked() const { return lock_; }

    [[nodiscard]] TargetPoint target() const { return target_; }

    void clearTrajectory()
    {
        if constexpr (!Offline)
        {
            ISRGuard lock{};
            cmd_buffer_.clear();
        }
    }

    [[nodiscard]] float getCurrentAvePosition() const
    {
        float sum = 0.0f;
        for (auto& ctrl : ctrl_)
            sum += ctrl->getMotor()->getAngle();
        return sum / static_cast<float>(MotorNum);
    }

    [[nodiscard]] float getCurrentAveVelocity() const
    {
        float sum = 0.0f;
        for (auto& ctrl : ctrl_)
            sum += ctrl->getMotor()->getVelocity();
        return sum / static_cast<float>(MotorNum);
    }

    template <size_t N = MotorNum, typename = std::enable_if_t<N == 1>>
    [[nodiscard]] float getCurrentPosition() const
    {
        return ctrl_[0]->getMotor()->getAngle();
    }

    template <size_t N = MotorNum, typename = std::enable_if_t<N == 1>>
    [[nodiscard]] float getCurrentVelocity() const
    {
        return ctrl_[0]->getMotor()->getVelocity();
    }

protected:
    controllers::MotorVelController* ctrl_[MotorNum]{};

    void updateTarget(const TargetPoint& target) { target_ = target; }

private:
    static constexpr float dps2rpm(const float dps) { return dps / 360.0f * 60.0f; }

    void applyTargetWithLastError()
    {
        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i]->setRef(dps2rpm(target_.dq + pd_[i].getOutput()));
    }

    bool enabled_{ false };
    bool lock_{ false };
    bool stopped_{ true };

    PD pd_[MotorNum]{};

    TargetPoint target_{};

    /// 当 Offline = true 时替代 RingBuffer 的空占位类型。
    struct NoBuffer
    {
    };

    [[no_unique_address]] std::conditional_t<Offline, NoBuffer,
                                             libs::RingBuffer<TargetPoint, BufferCapacity>>
            cmd_buffer_;
};

} // namespace trajectory
