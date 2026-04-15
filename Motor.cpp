#pragma once
#include "can.hpp"
#include "libxr.hpp"

class Motor {
 public:
  enum ControlMode { MODE_CURRENT, MODE_POSITION, MODE_VELOCITY };
  struct Feedback {
    float position;
    float velocity;
    float current;
    float abs_angle;
    float omega;
  };
  struct MotorCmd {
    ControlMode mode;
    float current;
    float position;
    float velocity;
  };

  virtual ~Motor() = default;
  virtual void Update() = 0;
  virtual const Feedback& GetFeedback() = 0;
  virtual void Control(const MotorCmd& cmd) = 0;
};

class GM6020 : public Motor {
 public:
  GM6020(LibXR::CAN* can_bus, uint16_t can_id = 1);
  void Update() override {}
  const Feedback& GetFeedback() override { return fb_; }
  void Control(const MotorCmd& cmd) override;

  static void SendAllPackets(LibXR::CAN* can);

 private:
  void OnCanMsg(const LibXR::CAN::ClassicPack& pack);
  uint16_t can_id_;
  LibXR::CAN* can_;
  Feedback fb_{};
  static int16_t can_tx_buffer[8];
};
