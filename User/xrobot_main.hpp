#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "Motor.hpp"
#include "PositionAction.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}