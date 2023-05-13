#pragma once

#include "serial_core.h"

namespace ROSControl
{
  class ROSBridge
  {
  public:
    ROSBridge();
    ~ROSBridge() = default;

    void recieveCommands();
    void sendReturns();
    void setReturns();

    ROSControl::Recieved getRecieved() const;

  private:
    std::unique_ptr<ROSControl::SerialCore> sprinter_;
    ROSControl::Recieved recieved_;
    ROSControl::Returns returns_;
  };
} // namespace ROSControl