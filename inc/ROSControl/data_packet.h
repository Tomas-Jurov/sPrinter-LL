#ifndef SPRINTER_DATA_PACKET_H
#define SPRINTER_DATA_PACKET_H
#include <cstdint>
namespace ROSControl
{
  enum HeaderFunc
  {
    READ_RETURNS = 0x69,
    WRITE_PARAMS = 0x70
  };

  struct Header
  {
    uint8_t header_func;
    uint16_t data_len;
    uint8_t command;
  } __attribute__((packed));

  union DataPacket
  {
      Header header;
      uint8_t messsage[266];
  };

  enum Command
  {
    SET_SPEED_OF_WHEELS =       0x00,
    SET_SPEED_OF_LIN_ACTUATOR = 0x01,
    SET_STEPPER1_SPEED =        0x02,
    SET_STEPPER2_SPEED =        0x03,
    SET_STEPPER1_TARG_STEPS =   0x04,
    SET_STEPPER2_TARG_STEPS =   0x05,
    SET_SERVO1_TARG_ANGLE =     0x06,
    SET_SERVO2_TARG_ANGLE =     0x07,
    START_SUNTRACKING =         0x08,
	RESET =                     0x09
  };

  struct WheelsVelocity
  {
    short left  : 8;
    short right : 8;
  } __attribute__((packed)); 

  struct Returns
  {
    short left_grp_vel          : 8;
    short right_grp_vel         : 8;
    long stepper1_current_steps : 32;
    long stepper2_current_steps : 32;
    short servo1_current_angle  : 16;
    short servo2_current_angle  : 16;
    bool suntracker_done        : 1;
    bool tilt_is_on_point       : 1;
  } __attribute__((packed));

  struct Recieved
  {
    unsigned short command : 8;
    WheelsVelocity wheels_vel;
    int8_t tilt_vel;
    uint16_t stepper1_speed;
    uint16_t stepper2_speed;
    int32_t stepper1_target;
    int32_t stepper2_target;
    uint16_t servo1_angle;
    uint16_t servo2_angle;
  } __attribute__((packed));

} // namespace ROSControl


#endif //SPRINTER_DATA_PACKET_H
