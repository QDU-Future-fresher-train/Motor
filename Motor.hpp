#pragma once
// clang-format on

#include <cstdint>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief 电机抽象接口
 * @details 统一描述电机控制命令、反馈格式与基础控制能力。
 */
class Motor {
 public:
  enum ControlMode : uint8_t {
    MODE_POSITION, /*位置控制模式*/
    MODE_VELOCITY, /*速度控制模式*/
    MODE_TORQUE,   /*力矩控制模式*/
    MODE_CURRENT,  /*电流控制模式*/
    MODE_MIT,      /*MIT控制模式*/
  };

  struct MotorCmd {
    ControlMode mode;             /*电机控制模式*/
    float reduction_ratio = 1.0f; /*减速比*/
    float torque = 0;             /*力矩*/
    float position = 0;           /*目标位置*/
    float velocity = 0;           /*目标速度*/
    float kp = 0;                 /*刚度*/
    float kd = 0;                 /*阻尼*/
  };

  struct Feedback {
    uint8_t error_id;                   /*电机错误码*/
    uint8_t state = 0;                  /*电机错误状态*/
    float position;                     /*电机原始角度*/
    LibXR::CycleValue<float> abs_angle; /*CycleValue的角度*/
    float velocity;                     /*转速*/
    float omega;                        /*角速度*/
    float torque;                       /*扭矩*/
    float temp;                         /*温度*/
  };

  /**
   * @brief 虚析构函数
   */
  virtual ~Motor() = default;

  /**
   * @brief 使能电机输出
   */
  virtual void Enable() = 0;

  /**
   * @brief 失能电机输出
   */
  virtual void Disable() = 0;

  /**
   * @brief 松开电机（输出零力矩/零电流）
   */
  virtual void Relax() = 0;

  /**
   * @brief 更新电机反馈
   * @return ErrorCode 更新结果
   */
  virtual ErrorCode Update() = 0;

  /**
   * @brief 获取当前反馈
   * @return const Feedback& 反馈数据引用
   */
  virtual const Feedback& GetFeedback() = 0;

  /**
   * @brief 下发控制命令
   * @param cmd 控制命令
   */
  virtual void Control(const MotorCmd& cmd) = 0;

  /**
   * @brief 清除电机错误状态
   */
  virtual void ClearError() = 0;

  /**
   * @brief 保存零点
   */
  virtual void SaveZeroPoint() = 0;

 private:
};
