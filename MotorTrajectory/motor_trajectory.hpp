/**
 * @file    motor_trajectory.hpp
 * @author  syhanjin
 * @date    2026-01-29
 */
#ifndef MOTOR_TRAJECTORY_HPP
#define MOTOR_TRAJECTORY_HPP
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include "cstddef"
#include <array>

/**
 * deg/s 转为 rpm
 * @param __DEG_PER_SEC__ deg/s
 */
#define DPS2RPM(__DEG_PER_SEC__) ((__DEG_PER_SEC__) / 360.0f * 60.0f)

namespace trajectory
{

template <size_t MotorNum> class MotorTrajectory
{
public:
    MotorTrajectory(controllers::MotorVelController*        motor_controllers[MotorNum],
                    velocity_profile::SCurveProfile::Config profile_cfg,
                    const PD::Config&                       error_pd_cfg);

    void profileUpdate(float dt);
    void errorUpdate();
    void controllerUpdate();

    void stop();
    bool setTarget(const float& target);
    bool setRelativeTarget(const float& target);

    float getCurrentAvePosition() const;
    float getCurrentAveVelocity() const;

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

    void disable()
    {
        stop();
        for (auto& ctrl : ctrl_)
            ctrl->disable();
    }

    bool enabled() const { return enabled_; }

    void lock() { lock_ = true; }
    void unlock() { lock_ = false; }
    bool locked() const { return lock_; }

private:
    bool enabled_{ false };
    bool lock_{ false };
    bool stopped_{ true };

    controllers::MotorVelController* ctrl_[MotorNum]{};

    PD pd_[MotorNum]{};

    velocity_profile::SCurveProfile::Config profile_cfg_;
    velocity_profile::SCurveProfile         profile_;

    float p_ref_curr_{ 0 };
    float v_ref_curr_{ 0 };
    float v_ref_curr_rpm_{ 0 };

    float now_{ 0 };
};

template <size_t MotorNum>
MotorTrajectory<MotorNum>::MotorTrajectory(
        controllers::MotorVelController*        motor_controllers[MotorNum],
        velocity_profile::SCurveProfile::Config profile_cfg,
        const PD::Config&                       error_pd_cfg) :
    profile_cfg_(profile_cfg), profile_(profile_cfg, 0, 0, 0, 0)
{
    for (size_t i = 0; i < MotorNum; ++i)
        ctrl_[i] = motor_controllers[i];

    for (auto& p : pd_)
        p.setConfig(error_pd_cfg);
}

/**
 * 更新速度规划曲线
 * @param dt 间隔时间 (unit: s)
 */
template <size_t MotorNum> void MotorTrajectory<MotorNum>::profileUpdate(const float dt)
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
template <size_t MotorNum> void MotorTrajectory<MotorNum>::errorUpdate()
{
    if (!enabled() || locked())
        return;
    for (size_t i = 0; i < MotorNum; ++i)
        ctrl_[i]->setRef(
                DPS2RPM(v_ref_curr_ + pd_[i].calc(p_ref_curr_, ctrl_[i]->getMotor()->getAngle())));
}

/**
 * 更新速度环
 */
template <size_t MotorNum> void MotorTrajectory<MotorNum>::controllerUpdate()
{
    if (!enabled() || locked())
        return;
    for (auto& ctrl : ctrl_)
        ctrl->update();
}

template <size_t MotorNum> void MotorTrajectory<MotorNum>::stop()
{
    stopped_    = true;
    p_ref_curr_ = getCurrentAvePosition();
    v_ref_curr_ = 0;
}

template <size_t MotorNum> bool MotorTrajectory<MotorNum>::setTarget(const float& target)
{
    if (!enabled() || locked())
        return false;

    // try to construct profile
    const velocity_profile::SCurveProfile p(profile_cfg_,
                                            getCurrentAvePosition(),
                                            getCurrentAveVelocity(),
                                            profile_.CalcA(now_),
                                            target);
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

template <size_t MotorNum> bool MotorTrajectory<MotorNum>::setRelativeTarget(const float& target)
{
    if (!enabled() || locked())
        return false;

    // try to construct profile
    const velocity_profile::SCurveProfile p(profile_cfg_,
                                            getCurrentAvePosition(),
                                            getCurrentAveVelocity(),
                                            profile_.CalcA(now_),
                                            getCurrentAvePosition() + target);
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
template <size_t MotorNum> float MotorTrajectory<MotorNum>::getCurrentAvePosition() const
{
    float sum = 0;
    for (auto& ctrl : ctrl_)
        sum += ctrl->getMotor()->getAngle();
    return sum / MotorNum;
}
template <size_t MotorNum> float MotorTrajectory<MotorNum>::getCurrentAveVelocity() const
{
    float sum = 0;
    for (auto& ctrl : ctrl_)
        sum += ctrl->getMotor()->getVelocity();
    return sum / MotorNum;
}

} // namespace trajectory

#endif // MOTOR_TRAJECTORY_HPP
