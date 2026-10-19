# TrajectoryControl

基于速度曲线的电机轨迹控制与堵转归零模块。仓库作为宿主 STM32 项目的 Git SubModule 使用，由 cpkg 解析包依赖并接入宿主构建，不提供独立固件工程。

## 包导航

| 包目录 | cpkg 包名 | 职责 |
| --- | --- | --- |
| [MotorTrajectory](MotorTrajectory/README.md) | `Trajectory::MotorTrajectory` | 本地 S 曲线规划、多电机位置误差补偿与速度环调度；也支持跟踪外部轨迹点。 |
| [HomingMotorTrajectory](HomingMotorTrajectory/README.md) | `Trajectory::HomingMotorTrajectory` | 在基础轨迹上增加堵转寻零、逻辑零位偏移及归零后定位。 |

## 集成边界

- 轨迹层使用外部电机控制器及其反馈，不拥有控制器或电机资源；宿主负责生命周期、反馈刷新、通信与周期调度。
- 本地曲线由 `VelocityProfile::SCurve` 提供，底层速度控制由 `MotorDrivers::Controller` 提供；本仓库不替代曲线求解包或电机驱动。
- 归零包额外依赖宿主 CubeMX/HAL1 环境及 `HAL_GetTick()`。硬件静态配置由宿主 CubeMX 承担，运行期归零不等同于硬件初始化或安全保护。
- 包的依赖、调用顺序、失败和资源边界见各包 README；符号说明及已有 Doxygen 见包内头文件。本文不构成所有 STM32 系列的兼容性或实机验证保证。
