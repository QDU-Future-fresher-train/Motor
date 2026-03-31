# Motor

用于统一电机控制接口的抽象基类模块，业务模块通过它屏蔽具体电机驱动差异（如 `RMMotor`、`DMMotor`）。

## 解决什么问题

Motor 主要解决三件事：

1. 统一控制命令结构（位置/速度/力矩/电流/MIT）。
2. 统一反馈结构（角度、角速度、扭矩、温度、错误码）。
3. 让上层模块只依赖抽象接口，不耦合具体驱动实现。

## 核心接口

`Motor` 是纯虚接口，核心方法如下：

1. 生命周期：`Enable()` / `Disable()` / `Relax()`。
2. 闭环更新：`Update()`。
3. 反馈读取：`GetFeedback()`。
4. 控制下发：`Control(const MotorCmd&)`。
5. 维护接口：`ClearError()` / `SaveZeroPoint()`。

核心数据结构：

1. `Motor::ControlMode`：`MODE_POSITION`、`MODE_VELOCITY`、`MODE_TORQUE`、`MODE_CURRENT`、`MODE_MIT`。
2. `Motor::MotorCmd`：统一控制指令参数。
3. `Motor::Feedback`：统一反馈数据。

## 典型使用方式

业务层建议只持有 `Motor*`，例如：

```cpp
void DriveMotor(Motor* motor) {
  motor->Enable();
  motor->Update();

  Motor::MotorCmd cmd{};
  cmd.mode = Motor::MODE_VELOCITY;
  cmd.velocity = 30.0f;
  cmd.reduction_ratio = 1.0f;
  motor->Control(cmd);
}
```

## 新电机驱动如何接入

假设你要新增一个驱动 `MyMotor`，建议流程：

1. `class MyMotor : public Motor`。
2. 实现全部纯虚接口。
3. 在 `Control()` 内按 `ControlMode` 分发到底层协议。
4. 在 `Update()` 内更新并维护 `Feedback`。
5. 上层模块继续使用 `Motor*`，无需改业务逻辑。

## 使用约定

1. `Update()` 与 `Control()` 建议在固定周期调用。
2. 控制前先确认 `Enable()` 已执行且反馈在线。
3. `Feedback::abs_angle` 用于跨圈角度场景，避免直接用原始单圈角。

## 模块信息

1. 代码入口：`Modules/Motor/Motor.hpp`
2. Required Hardware：None
3. Constructor Arguments：None
4. Template Arguments：None
5. Depends：None
sssssss