#include "ROSControl/ros_bridge.h"

namespace ROSControl {

  ROSBridge::ROSBridge()
  : sprinter_(std::make_unique<SerialCore>())
  {
  }

  void ROSBridge::recieveCommands()
  {
    sprinter_->readRecieved(&recieved_);
  }

  void ROSBridge::sendReturns()
  {
    sprinter_->sendState(returns_);
  }

  Recieved ROSBridge::getRecieved() const
  {
  return recieved_;
  }

  void ROSBridge::setReturns()
  {
    returns_.left_grp_vel = 5;
    returns_.right_grp_vel = 6;
    returns_.servo1_current_angle = 7;
    returns_.servo2_current_angle = 4;
    returns_.stepper1_current_steps = 50;
    returns_.stepper2_current_steps = 100;
    returns_.suntracker_done = true;
  }
}
