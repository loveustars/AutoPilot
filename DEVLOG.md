# ADAS Development Log | 开发日志

## 错误记录 | Error Log

| 日期 | 模块 | 错误描述 | 解决方案 | 状态 |
|------|------|----------|----------|------|
| 2026-01-17 | Setup | 项目框架初始化 | - | ✅ |
| 2026-01-17 | Build | CMake找不到adas_msgs包 | 使用`<depend>`替代分离的build/exec依赖 | ✅ |
| 2026-01-17 | Build | catkin_package()找不到include目录 | 移除不存在的include目录引用 | ✅ |
| 2026-01-17 | Launch | `$(param ...)` 无效substitution命令 | 改用 `<rosparam>` 直接加载到节点 | ✅ |
| 2026-01-17 | Python | Python 3.8 type annotation 错误 | 添加 `from __future__ import annotations` | ✅ |
| 2026-01-17 | Runtime | carla_msgs import失败 | 需同时source carla-ros-bridge workspace | ✅ |
| 2026-01-17 | Runtime | 车辆控制topic名称错误 | `/carla/hero/...` → `/carla/ego_vehicle/...` | ✅ |
| 2026-01-17 | Runtime | Safety Monitor启动时误报超时 | 等待第一个heartbeat后再激活watchdog | ✅ |
| 2026-01-17 | Build | PYTHONPATH未正确叠加 | 重新编译时先source carla-ros-bridge作为overlay | ✅ |
| 2026-01-17 | Config | watchdog 50ms在仿真中太紧 | 仿真环境改为100ms超时 | ✅ |
| 2026-01-17 | Perception | LiDAR地面过滤错误 | CARLA坐标系z负值是下方，修改为z>-1.8的点为障碍物 | ✅ |
| 2026-01-17 | AEB | 响应速度慢，正常行驶误触发 | 重新调整TTC阈值、心跳频率、高度容忍度 | ✅ |
| 2026-01-17 | Safety | 需要紧急情况下覆盖用户操作 | 实现Emergency Override功能 | ✅ |
| 2026-01-17 | AEB | 静止时频繁误报WARNING/FULL_BRAKE | 添加速度阈值、障碍物稳定性、点数过滤 | ✅ |
| 2026-01-17 | Control | Emergency Override无法覆盖手动控制 | 发布到vehicle_control_manual_override禁用手动模式 | ✅ |
| 2026-01-17 | ROS-Bridge | numpy.bool废弃导致DVS相机崩溃 | 修改camera.py的numpy.bool为numpy.bool_ | ✅ |

---

## 变更记录 | Change Log

### 2026-01-17 - AEB误报修复与Override机制完善
- [x] **静止状态保护**：
  - 添加`min_ego_velocity=0.5m/s`阈值，低于此速度不计算TTC
  - 避免静止时噪声点导致的误报
- [x] **障碍物稳定性检测**：
  - 需要连续3帧检测到同一障碍物才计算TTC
  - 添加`min_obstacle_confidence=0.3`过滤低置信度检测
- [x] **状态确认机制**：
  - AEB FSM需要连续2帧确认才能切换状态
  - FULL_BRAKE紧急状态仍然立即响应（1帧）
  - 退出高优先级状态需要0.5s最小驻留时间
- [x] **LiDAR噪声过滤**：
  - 添加`min_obstacle_points=15`过滤小噪声
  - 添加`min_obstacle_size=0.3m`过滤小于行人的物体
- [x] **CARLA手动控制覆盖**：
  - 发布`False`到`vehicle_control_manual_override`话题
  - 禁用manual_control.py的控制权，AEB获得完全控制
  - 紧急状态结束后自动恢复手动控制
- [x] **numpy.bool修复**：
  - 修改carla-ros-bridge/camera.py中的`numpy.bool`为`numpy.bool_`

### 2026-01-17 - AEB系统优化
- [x] **TTC阈值重新调整**：
  - WARNING: TTC 1.5s~3.0s（原2.5s~4.0s）
  - PARTIAL_BRAKE: TTC 1.0s~1.5s（原1.5s~2.5s）
  - FULL_BRAKE: TTC <1.0s（原<1.5s）- 最大制动(brake=1.0)
- [x] **心跳频率提升**：40Hz (25ms周期) ← 原50Hz
- [x] **Watchdog超时缩短**：50ms ← 原100ms
- [x] **地面过滤高度容忍度**：ground_z_max=-1.5 ← 原-1.8 (增加0.3m容忍)
- [x] **检测ROI收窄**：min_y/max_y=±8m ← 原±10m
- [x] **Emergency Override实现**：
  - TTC < 1.0s时自动覆盖用户手动控制
  - vehicle_controller订阅AEB状态
  - FULL_BRAKE状态强制brake=1.0

### 2026-01-17 - 项目初始化
- [x] 创建catkin工作空间结构
- [x] 初始化adas_msgs消息包
- [x] 初始化adas_perception感知模块
- [x] 初始化adas_decision决策模块
- [x] 初始化adas_safety安全监控模块
- [x] 初始化adas_control控制模块
- [x] 初始化adas_hal硬件抽象层
- [x] 初始化adas_bringup系统启动包

---

## 待解决问题 | Known Issues

- [] 行进时极易误判导致无法移动，暂未修复

---

## 性能基准 | Performance Baseline

| 测试项 | 目标 | 实测 | 状态 |
|--------|------|------|------|
| 端到端延迟 | <100ms | - | 🔄 |
| 心跳检测 | 25ms | 40Hz配置 | ✅ |
| Watchdog超时 | 50ms | 配置完成 | ✅ |
| 点云处理 | <30ms | - | 🔄 |
| AEB响应(TTC<1s) | 立即全刹 | Emergency Override | ✅ |

---

## 调试笔记 | Debug Notes

### AEB调优备忘
- **CARLA坐标系**：z轴负值向下，LiDAR约在2.4m高度
- **地面过滤关键**：`ground_z_max=-1.5`表示保留z>-1.5的点作为障碍物
- **FULL_BRAKE**：直接设置`brake=1.0`而非计算减速度，确保最大制动力
- **Emergency Override**：在vehicle_controller中订阅AEB状态，TTC<1s时覆盖用户输入

_在此记录调试过程中的发现..._
