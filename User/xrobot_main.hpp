#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "Action.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static RMMotor rmmotor(hw, appmgr, None);

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}