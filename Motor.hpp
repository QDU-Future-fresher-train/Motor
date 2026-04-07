#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: GM6020电机驱动
constructor_args: []
template_args: []
required_hardware:
  - param
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "mutex.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/*
   GM6020 电机 CAN ID 定义（来自大疆官方协议）
 */
#define GM6020_FB_ID_BASE     (0x205)     // 反馈ID 起始：1~4号电机
#define GM6020_FB_ID_EXTAND   (0x209)     // 扩展：5~7号电机（注意只有到0x20B）
#define GM6020_CTRL_ID_BASE   (0x1fe)     // 控制ID：控制1~4号电机
#define GM6020_CTRL_ID_EXTAND (0x2fe)     // 控制ID：控制5~7号电机

#define MOTOR_CTRL_ID_NUMBER  (4)         // 每组控制报文最多控制4个电机

/*
   GM6020 关键物理/协议参数
 */
#define GM6020_MAX_ABS_LSB    (16384)     // 控制量最大绝对值（-16384 ~ +16384）
#define GM6020_MAX_ABS_CUR    (3)         // 最大允许电流 ≈ ±3A（官方手册）

#define MOTOR_ENC_RES         (8192)      // 编码器分辨率：8192个脉冲/圈（机械角度）
#define MOTOR_CUR_RES         (16384)     // 电流反馈分辨率：16384 LSB = 最大电流范围

/*
   反馈结构体（用户最常读取的内容）
*/
struct Feedback {
    float position;     // 当前机械角度（rad，范围通常0~2π，可多圈累加）
    float velocity;     // 转速（RPM，原始协议单位）
    float abs_angle;    // 归一化到 [0, 2π) 的绝对角度（使用 CycleValue 实现）
    float omega;        // 角速度（rad/s）
    float torque;       // 输出扭矩估计值（N·m）
    float temp;         // 电机温度（℃）
    uint8_t state;      // 电机状态（这里简单用1表示在线）
};

/*
   对外控制指令模式
 */
enum class ControlMode : uint8_t {
    MODE_TORQUE = 0,     // 力矩控制（N·m）
    MODE_CURRENT         // 电流控制
};


struct MotorCmd {
    ControlMode mode;
    union {
        struct {
            float torque;           // 期望输出轴扭矩 (N·m)
            float reduction_ratio;  // 减速比（输出轴扭矩 = 电机扭矩 × 减速比）
        };
        float velocity;             // 期望归一化电流/电压（MODE_CURRENT时使用）
    };
};

/*
   GM6020 电机驱动类（继承自应用框架的 Application）
 */
class Motor : public LibXR::Application {
public:
    struct Param {
        bool reverse;
        uint16_t feedback_id;
        const char* can_bus_name;
  };

    /* 内部使用的配置（根据 feedback_id 自动推导） */
    struct ConfigParam {
        uint32_t id_feedback;   // 接收的 CAN ID
        uint32_t id_control;    // 发送的控制 CAN ID（0x1fe 或 0x2fe）
    };

    /* 静态共享缓冲区
       所有同组电机（同一条0x1fe或0x2fe）共用一个8字节缓冲区
       每个电机占2字节（int16_t电流值）
    */
    static inline uint8_t  motor_tx_buff_[2][4][8]{};       // [can_idx][group][data]
    static inline uint8_t  motor_tx_flag_[2][4]{};          // 位图：哪几个电机已更新
    static inline uint8_t  motor_tx_map_[2][4]{};           // 位图：本组有哪些电机存在

public:
    /**
     * @brief 构造函数
     * @param hw        硬件容器（从中找 CAN 实例）
     * @param app       应用管理器（本例未使用）
     * @param param     电机配置参数
     */
    Motor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const Param& param)
        : param_(param),
          can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
        UNUSED(app);
        reverse_flag_ = param_.reverse ? -1.0f : 1.0f;

        // 根据反馈ID确定控制组ID（这是GM6020协议规定的）
        if (param_.feedback_id >= 0x205 && param_.feedback_id <= 0x208) {
            config_param_.id_control  = GM6020_CTRL_ID_BASE;
            config_param_.id_feedback = param_.feedback_id;
        } else if (param_.feedback_id >= 0x209 && param_.feedback_id <= 0x20B) {
            config_param_.id_control  = GM6020_CTRL_ID_EXTAND;
            config_param_.id_feedback = param_.feedback_id;
        } else {
            // 非法ID，置为0，后续会失效
            config_param_.id_control  = 0;
            config_param_.id_feedback = 0;
        }

        // 计算本电机在控制组中的位置（0~3）
        uint8_t motor_num   = 0;
        uint8_t motor_index = 0;

        switch (config_param_.id_control) {
            case GM6020_CTRL_ID_BASE:
                motor_index = 2;  // 第一组统一用索引2
                motor_num   = config_param_.id_feedback - GM6020_FB_ID_BASE;
                break;
            case GM6020_CTRL_ID_EXTAND:
                motor_index = 3;  // 第二组统一用索引3
                motor_num   = config_param_.id_feedback - GM6020_FB_ID_EXTAND;
                break;
            default:
                motor_index = 0;
                motor_num   = 0;
                break;
        }

        index_ = motor_index;
        num_   = motor_num;

        // 解析 can1 / can2
        if (const char* p = std::strpbrk(param_.can_bus_name, "12")) {
            can_index_ = *p - '1';  // 0 → can1, 1 → can2
        } else {
            can_index_ = 0xFF;      // 非法总线
        }

        // 标记这个电机存在（用于判断是否凑齐一组）
        motor_tx_map_[can_index_][motor_index] |= (1 << motor_num);

        // 清空发送缓冲
        memset(motor_tx_buff_[can_index_][index_], 0,
               sizeof(motor_tx_buff_[can_index_][index_]));

        // 注册 CAN 接收回调（只接收本电机反馈ID）
        auto rx_callback = LibXR::CAN::Callback::Create(
            [](bool in_isr, Motor* self, const LibXR::CAN::ClassicPack& pack) {
                RxCallback(in_isr, self, pack);
            },
            this);

        can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                       LibXR::CAN::FilterMode::ID_RANGE,
                       config_param_.id_feedback,
                       config_param_.id_feedback);
    }

    /* 接口函数 ──────────────────────────────────────── */

    /**
     * @brief 主循环中调用，处理接收到的反馈
     */
    ErrorCode Update() {
        LibXR::CAN::ClassicPack pack;
        while (recv_queue_.Pop(pack) == ErrorCode::OK) {
            this->Decode(pack);
        }
        return ErrorCode::OK;
    }

    const Feedback& GetFeedback() { return feedback_; }

    /**
     * @brief 统一控制接口（力矩 or 电流）
     */
    void Control(const MotorCmd& cmd) {
        switch (cmd.mode) {
            case ControlMode::MODE_TORQUE:
                this->TorqueControl(cmd.torque, cmd.reduction_ratio);
                break;
            case ControlMode::MODE_CURRENT:
                this->CurrentControl(cmd.velocity);
                break;
            default:
                break;
        }
    }
      void OnMonitor() override {}

private:
    uint8_t can_index_{};       // 0=can1, 1=can2
    uint8_t index_;             // 控制组：2=0x1fe组, 3=0x2fe组
    uint8_t num_;               // 组内电机编号 0~3

    float reverse_flag_ = 1.0f; // -1.0 = 反转

    Param       param_;
    ConfigParam config_param_;
    Feedback    feedback_{};

    LibXR::CAN* can_ = nullptr;

    // 中断安全接收队列（防止丢失反馈）
    LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};

    LibXR::Mutex mutex_;   // 保护共享发送缓冲区（虽然一般中断里不发，但以防万一）

private:
    /**
     * @brief 发送一组控制报文（8字节包含4个电机的电流指令）
     * @return 总是返回 true
     */
    bool SendData() {
        LibXR::CAN::ClassicPack tx_pack{};
        tx_pack.id   = config_param_.id_control;
        tx_pack.type = LibXR::CAN::Type::STANDARD;

        // 拷贝共享缓冲区内容
        LibXR::Memory::FastCopy(tx_pack.data,
                                motor_tx_buff_[can_index_][index_],
                                sizeof(tx_pack.data));

        can_->AddMessage(tx_pack);

        // 清标志位 & 清缓冲
        motor_tx_flag_[can_index_][index_] = 0;
        memset(motor_tx_buff_[can_index_][index_], 0,
               sizeof(motor_tx_buff_[can_index_][index_]));

        return true;
    }

    /**
     * @brief CAN 中断接收回调（静态函数）
     *        把收到的包放入无锁队列，队列满就丢弃最旧的
     */
    static void RxCallback(bool in_isr, Motor* self,
                           const LibXR::CAN::ClassicPack& pack) {
        UNUSED(in_isr);
        // 队列满就丢弃旧数据，保证最新数据能进
        while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
            self->recv_queue_.Pop();
        }
    }

    /**
     * @brief 解析 GM6020 标准反馈协议（8字节）
     *        协议格式（大疆官方）：
     *   [0~1] 机械角度    uint16   0~8191 → 一圈
     *   [2~3] 转速        int16    RPM
     *   [4~5] 转矩电流    int16    LSB
     *   [6]   温度        uint8    ℃
     *   [7]   保留
     */
    void Decode(LibXR::CAN::ClassicPack& pack) {
        uint16_t raw_angle   = (pack.data[0] << 8) | pack.data[1];
        int16_t  raw_velocity = (pack.data[2] << 8) | pack.data[3];
        int16_t  raw_current  = (pack.data[4] << 8) | pack.data[5];
        uint8_t  raw_temp     = pack.data[6];

        // 根据正反转标志处理符号
        if (param_.reverse) {
            feedback_.position = -static_cast<float>(raw_angle) / MOTOR_ENC_RES * M_2PI;
            feedback_.velocity = static_cast<float>(-raw_velocity);
        } else {
            feedback_.position = static_cast<float>(raw_angle) / MOTOR_ENC_RES * M_2PI;
            feedback_.velocity = static_cast<float>(raw_velocity);
        }

        // 归一化到 [0, 2π)
        feedback_.abs_angle = LibXR::CycleValue<float>(feedback_.position);

        // 角速度 rad/s
        feedback_.omega = feedback_.velocity * (M_2PI / 60.0f);

        // 扭矩估算：电流(LSB) → 实际电流(A) → 扭矩(N·m)
        feedback_.torque = static_cast<float>(raw_current) *
                           KGetTorque() * GetCurrentMAX() / MOTOR_CUR_RES;

        feedback_.temp  = static_cast<float>(raw_temp);
        feedback_.state = 1;  // 默认认为在线
    }

    /**
     * @brief 力矩控制（输出轴扭矩 N·m）
     * @param torque         期望输出轴扭矩
     * @param reduction_ratio 减速比（>1 表示减速）
     */
    void TorqueControl(float torque, float reduction_ratio) {
        // 过温保护（硬保护）
        if (feedback_.temp > 75.0f) {
            torque = 0.0f;
            XR_LOG_WARN("motor %d high temperature detected", param_.feedback_id);
        }

        // 电机端扭矩 = 输出扭矩 / 减速比
        // 再除以 Kt 和 Imax 得到归一化电流比例
        float output = std::clamp(torque / reduction_ratio / KGetTorque() / GetCurrentMAX(),
                                  -1.0f, 1.0f) *
                       GetLSB() * reverse_flag_;

        int16_t ctrl_cmd = static_cast<int16_t>(output);

        // 写入共享缓冲区（每个电机占2字节）
        motor_tx_buff_[can_index_][index_][2 * num_ + 0] = (ctrl_cmd >> 8) & 0xFF;
        motor_tx_buff_[can_index_][index_][2 * num_ + 1] = ctrl_cmd & 0xFF;

        // 标记本电机已更新
        motor_tx_flag_[can_index_][index_] |= (1 << num_);

        // 如果本组所有存在的电机都更新了 → 立即发送
        if (((~motor_tx_flag_[can_index_][index_]) & motor_tx_map_[can_index_][index_]) == 0) {
            SendData();
        }

    }

    /**
     * @brief 电流/电压比例控制（-1.0 ~ 1.0）
     */
    void CurrentControl(float out) {
        if (can_index_ == 0xFF) return;

        // 过温保护
        if (feedback_.temp > 75.0f) {
            out = 0.0f;
            XR_LOG_WARN("motor %d high temperature detected", param_.feedback_id);
        }

        out = std::clamp(out, -1.0f, 1.0f);

        // 直接映射到 ±16384
        float output = out * GetLSB() * reverse_flag_;
        int16_t ctrl_cmd = static_cast<int16_t>(output);

        mutex_.Lock();

        motor_tx_buff_[can_index_][index_][2 * num_ + 0] = (ctrl_cmd >> 8) & 0xFF;
        motor_tx_buff_[can_index_][index_][2 * num_ + 1] = ctrl_cmd & 0xFF;

        motor_tx_flag_[can_index_][index_] |= (1 << num_);

        if (((~motor_tx_flag_[can_index_][index_]) & motor_tx_map_[can_index_][index_]) == 0) {
            SendData();
        }

        mutex_.Unlock();
    }

    /* 参数查询函数（GM6020 固定值） */
    float KGetTorque()    { return 0.741f; }          // 扭矩常数 Kt (N·m/A)
    float GetCurrentMAX() { return GM6020_MAX_ABS_CUR; }
    float GetLSB()        { return GM6020_MAX_ABS_LSB; }
};
