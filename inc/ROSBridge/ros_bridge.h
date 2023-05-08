#pragma once

#include "serial_core.h"

class ROSBridge
{
public:
  ROSBridge();
  ~ROSBridge() = default;

  void recieveCommands();
  void sendReturns();
  void setReturns();

  sprinter::Recieved getRecieved() const;

private:
  std::unique_ptr<sprinter::SerialCore> sprinter_;
  sprinter::Recieved recieved_;
  sprinter::Returns returns_;
};