/**
 * @file    motor_trajectory.hpp
 * @author  syhanjin
 * @date    2026-01-29
 */
#ifndef MOTOR_TRAJECTORY_HPP
#define MOTOR_TRAJECTORY_HPP
#include "cstddef"
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include <algorithm>
#include <array>

/**
 * deg/s 转为 rpm
 * @param __DEG_PER_SEC__ deg/s
 */
#define DPS2RPM(__DEG_PER_SEC__) ((__DEG_PER_SEC__) / 360.0f * 60.0f)

namespace trajectory
{

enum class LinkMode
{
    CurrentState,  // 使用当前状态衔接
    PreviousCurve, // 衔接之前曲线
};

template <size_t MotorNum> class MotorTrajectory
{
public:
    using ProfileConfig = velocity_profile::SCurveProfile::Config;

    MotorTrajectory(controllers::MotorVelController* motor_controllers[MotorNum],
                    const ProfileConfig&             profile_cfg,
                    const PD::Config&                error_pd_cfg) :
        default_profile_cfg_(profile_cfg), profile_(profile_cfg, 0, 0, 0, 0)
    {
        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i] = motor_controllers[i];

        for (auto& p : pd_)
            p.setConfig(error_pd_cfg);
    }

    // 允许直接传入实例数组，而不是指针数组
    MotorTrajectory(controllers::MotorVelController (&motors)[MotorNum],
                    ProfileConfig     profile_cfg,
                    const PD::Config& error_pd_cfg) :
        default_profile_cfg_(profile_cfg), profile_(profile_cfg, 0, 0, 0, 0)
    {
        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i] = &motors[i];

        for (auto& p : pd_)
            p.setConfig(error_pd_cfg);
    }

    template <size_t N = MotorNum, typename = std::enable_if_t<N == 1>>
    MotorTrajectory(controllers::MotorVelController* motor_controller,
                    const ProfileConfig&             profile_cfg,
                    const PD::Config&                error_pd_cfg) :
        ctrl_{ motor_controller }, pd_{ PD(error_pd_cfg) }, default_profile_cfg_(profile_cfg),
        profile_(profile_cfg, 0, 0, 0, 0)
    {
    }

    /**
     * 更新速度规划曲线
     * @param dt 间隔时间 (unit: s)
     */
    void profileUpdate(const float dt)
    {
        if (!enabled() || locked() || stopped_)
            return;
        now_ += dt;
        p_ref_curr_     = profile_.CalcX(now_);
        v_ref_curr_     = profile_.CalcV(now_);
        v_ref_curr_rpm_ = DPS2RPM(v_ref_curr_);

        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i]->setRef(DPS2RPM(v_ref_curr_ + pd_[i].getOutput()));
    }

    /**
     * 更新误差补偿
     */
    void errorUpdate()
    {
        if (!enabled() || locked())
            return;
        for (size_t i = 0; i < MotorNum; ++i)
            ctrl_[i]->setRef(DPS2RPM(v_ref_curr_ +
                                     pd_[i].calc(p_ref_curr_, ctrl_[i]->getMotor()->getAngle())));
    }

    /**
     * 更新速度环
     */
    void controllerUpdate()
    {
        if (!enabled() || locked())
            return;
        for (auto& ctrl : ctrl_)
            ctrl->update();
    }

    /// 紧急停止，立即停止并锁定位置
    void stop()
    {
        stopped_    = true;
        p_ref_curr_ = getCurrentAvePosition();
        v_ref_curr_ = 0;
    }

    /// 设置目标位置
    bool setTarget(const float target, const LinkMode link_mode, const ProfileConfig& config)
    {
        // TODO: 增加可选衔接方式

        if (!enabled() || locked())
            return false;

        float xs = 0, vs = 0, as = 0;
        if (stopped_)                                   // 停止状态
        {
            xs = getCurrentAvePosition();
            vs = 0;
            as = 0;
        }
        else if (link_mode == LinkMode::CurrentState)   // 使用当前状态衔接 
        {
            xs = getCurrentAvePosition();
            vs = getCurrentAveVelocity();
            as = 0;
        }
        else if (link_mode == LinkMode::PreviousCurve)  // 衔接之前曲线
        {
            xs = profile_.CalcX(now_);
            vs = profile_.CalcV(now_);
            as = profile_.CalcA(now_);
        }

        /// 限制初始速度和加速度
        float amin = -config.max_acc, amax = config.max_acc;
        if (vs < -config.max_spd)
        {
            amin = 0;
            vs   = -config.max_spd;
        }
        else if (vs > config.max_spd)
        {
            amax = 0;
            vs   = config.max_spd;
        }
        as = std::clamp(as, amin, amax);

        // try to construct profile
        const velocity_profile::SCurveProfile p(config, xs, vs, as, target);

        // if failed
        if (!p.success())
            // if failed, return false;
            return false;
        // if success, set profile and reset now
        lock(); // lock before writing
        profile_ = p;
        now_     = 0;
        stopped_ = false;
        unlock();
        return true;
    }
    bool setTarget(const float target)
    {
        return setTarget(target, LinkMode::CurrentState, default_profile_cfg_);
    }
    bool setTarget(const float target, const LinkMode link_mode)
    {
        return setTarget(target, link_mode, default_profile_cfg_);
    }
    bool setTarget(const float target, const ProfileConfig& config)
    {
        return setTarget(target, LinkMode::CurrentState, config);
    }

    /// 设置相对位移目标
    bool setRelativeTarget(const float          target,
                           const LinkMode       link_mode,
                           const ProfileConfig& config)
    {
        if (!enabled() || locked())
            return false;

        float base_pos = 0;

        // 根据衔接模式确定相对位移的基准点
        if (link_mode == LinkMode::PreviousCurve && !stopped_)
        {
            base_pos = profile_.CalcX(now_); // 以当前轨迹的理论位置为基准
        }
        else
        {
            base_pos = getCurrentAvePosition(); // 以当前实际反馈位置为基准 [cite: 6]
        }

        // 调用已有的 setTarget 逻辑完成规划 [cite: 4, 21]
        return setTarget(base_pos + target, link_mode, config);
    }
    bool setRelativeTarget(const float target)
    {
        return setRelativeTarget(target, LinkMode::CurrentState, default_profile_cfg_);
    }
    bool setRelativeTarget(const float target, const LinkMode link_mode)
    {
        return setRelativeTarget(target, link_mode, default_profile_cfg_);
    }
    bool setRelativeTarget(const float target, const ProfileConfig& config)
    {
        return setRelativeTarget(target, LinkMode::CurrentState, config);
    }

    /// 获取当前平均位置
    [[nodiscard]] float getCurrentAvePosition() const
    {
        float sum = 0;
        for (auto& ctrl : ctrl_)
            sum += ctrl->getMotor()->getAngle();
        return sum / MotorNum;
    }

    /// 获取当前平均速度
    [[nodiscard]] float getCurrentAveVelocity() const
    {
        float sum = 0;
        for (auto& ctrl : ctrl_)
            sum += ctrl->getMotor()->getVelocity();
        return sum / MotorNum;
    }

    /// 获取当前单个电机位置（仅当 MotorNum == 1 时有效）
    template <size_t N = MotorNum, typename = std::enable_if_t<N == 1>>
    [[nodiscard]] float getCurrentPosition() const
    {
        return ctrl_[0]->getPosition();
    }

    template <size_t N = MotorNum, typename = std::enable_if_t<N == 1>>
    [[nodiscard]] float getCurrentVelocity() const
    {
        return ctrl_[0]->getVelocity();
    }

    [[nodiscard]] float getTotalTime() const { return profile_.getTotalTime(); }

    /// 使能轨迹控制的所有电机控制器，如果有任何一个控制器使能失败，则全部禁用
    bool enable()
    {
        bool enabled = true;
        for (auto& ctrl : ctrl_)
            enabled &= ctrl->enable();
        if (!enabled)
            for (auto& ctrl : ctrl_)
                ctrl->disable();
        enabled_ = enabled;
        return enabled;
    }

    /// 禁用轨迹控制的所有电机控制器，并立即停止
    void disable()
    {
        stop();
        for (auto& ctrl : ctrl_)
            ctrl->disable();
        enabled_ = false;
    }

    [[nodiscard]] bool enabled() const { return enabled_; } // 返回当前使能状态

    void               lock() { lock_ = true; }             // 锁定以禁止更新，通常在设置新目标时使用
    void               unlock() { lock_ = false; }          // 解锁以允许更新
    [[nodiscard]] bool locked() const { return lock_; }     // 返回当前锁定状态

    [[nodiscard]] bool isFinished() const { return now_ >= profile_.getTotalTime(); }   // 判断当前轨迹是否已完成

    void setDefaultProfileConfig(const ProfileConfig& cfg) { default_profile_cfg_ = cfg; }  // 设置速度规划配置

private:
    bool enabled_{ false };     ///< 当前使能状态
    bool lock_{ false };        ///< 锁定状态，锁定后将禁止 profileUpdate、errorUpdate 和 controllerUpdate 的执行
    bool stopped_{ true };      ///< 停止状态，停止后将以当前实际位置为目标位置，并将速度设为 0

    controllers::MotorVelController* ctrl_[MotorNum]{};     ///< 电机控制器数组，支持多个电机的同步控制

    PD pd_[MotorNum]{};         ///< 每个电机对应一个 PD 控制器，用于计算误差补偿

    ProfileConfig                   default_profile_cfg_;   ///< 速度规划配置
    velocity_profile::SCurveProfile profile_;               ///< 当前的 S 曲线速度规划实例

    float p_ref_curr_{ 0 };         ///< 当前目标位置
    float v_ref_curr_{ 0 };         ///< 当前目标速度 (deg/s)
    float v_ref_curr_rpm_{ 0 };     ///< 当前目标速度 (rpm)

    float now_{ 0 };                ///< 当前规划时间 (uint: s)
};

} // namespace trajectory

#endif // MOTOR_TRAJECTORY_HPP
