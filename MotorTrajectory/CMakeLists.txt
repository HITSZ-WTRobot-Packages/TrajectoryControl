add_library(__Trajectory_MotorTrajectory INTERFACE)

target_include_directories(__Trajectory_MotorTrajectory INTERFACE
        .
)

# link dependencies if any
target_link_libraries(__Trajectory_MotorTrajectory INTERFACE VelocityProfile::SCurve)
target_link_libraries(__Trajectory_MotorTrajectory INTERFACE MotorDrivers::Controller)
target_link_libraries(__Trajectory_MotorTrajectory INTERFACE libs::PID_PD)

# alias for external use
add_library(Trajectory::MotorTrajectory ALIAS __Trajectory_MotorTrajectory)
