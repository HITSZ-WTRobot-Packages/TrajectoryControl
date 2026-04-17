add_library(__Trajectory_HomingMotorTrajectory INTERFACE)

target_include_directories(__Trajectory_HomingMotorTrajectory INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Trajectory_HomingMotorTrajectory INTERFACE MotorDrivers::Controller)
target_link_libraries(__Trajectory_HomingMotorTrajectory INTERFACE Trajectory::MotorTrajectory)
target_link_libraries(__Trajectory_HomingMotorTrajectory INTERFACE stm32cubemx)

# alias for external use
add_library(Trajectory::HomingMotorTrajectory ALIAS __Trajectory_HomingMotorTrajectory)
