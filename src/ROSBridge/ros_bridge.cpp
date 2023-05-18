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

  void ROSBridge::setReturns(const ROSControl::Returns &sprinter_state)
  {
    returns_.left_grp_vel = sprinter_state.left_grp_vel;
    returns_.right_grp_vel = sprinter_state.right_grp_vel;
    returns_.stepper1_current_steps = sprinter_state.stepper1_current_steps;
    returns_.stepper2_current_steps = sprinter_state.stepper2_current_steps;
    returns_.servo1_current_angle = sprinter_state.servo1_current_angle;
    returns_.servo2_current_angle = sprinter_state.servo2_current_angle;
    returns_.suntracker_done = sprinter_state.suntracker_done;
  }
}
