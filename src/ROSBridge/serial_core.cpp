#include "../inc/ROSBridge/serial_core.h"
#include <string.h>
sprinter::SerialCore::SerialCore()
: serial_port_(std::make_unique<Periph::Usart>(Periph::Usarts::Usart3, 230400))
{
}

bool sprinter::SerialCore::readRecieved(sprinter::Recieved *data)
{
  return readDataFromROS((bytePtr)data);
}

bool sprinter::SerialCore::sendState(const sprinter::Returns& data)
{
  return sendCurrentStateOfSprinter((constBytePtr)&data, sizeof(Returns));
}

bool sprinter::SerialCore::readDataFromROS(bytePtr data)
{
  DataPacket data_packet;
  
  size_t header_size = sizeof(data_packet.header);
  size_t crc_size = sizeof(uint32_t);

  int32_t read_header_bytes = serial_port_->read(data_packet.messsage, header_size);

  if (read_header_bytes != static_cast<int32_t>(header_size))
  {
    return 1;
  }
  size_t data_size =  static_cast<size_t>(data_packet.header.data_len);
  size_t expected_data_size = 0;
  if (data_packet.header.command == 0)
  {
    expected_data_size = sizeof(SpeedOfWheels);
  }
  size_t read_data_bytes = serial_port_->read(data_packet.messsage + header_size, data_size + crc_size); 

  if (read_data_bytes < (data_size + crc_size) || expected_data_size != data_size || (read_data_bytes - crc_size) != expected_data_size)
  {
    return 1;
  }

  if (data_packet.header.header_func != WRITE_PARAMS)
  {
    return 1;
  }

  uint32_t crc = crc32(data_packet.messsage, header_size + data_size);

  uint32_t data_packet_crc;
  memcpy(&data_packet_crc, data_packet.messsage + header_size + data_size, crc_size);
  if (crc != data_packet_crc)
  {
    return 1;
  }

  memcpy((bytePtr)data, data_packet.messsage + sizeof(uint8_t) + sizeof(uint16_t), sizeof(uint8_t));
  memcpy((bytePtr)data + sizeof(uint8_t), data_packet.messsage + header_size, data_size);  

  return 0;
}

bool sprinter::SerialCore::sendCurrentStateOfSprinter(constBytePtr data, size_t data_size)
{
  DataPacket data_packet;

  size_t header_size = sizeof(data_packet.header);
  size_t crc_size = sizeof(uint32_t);
  size_t data_packet_size = header_size + data_size + crc_size;

  data_packet.header.header_func = READ_RETURNS;
  data_packet.header.data_len = (uint16_t)data_size;
  data_packet.header.command = 0xFF;

  memcpy(data_packet.messsage + header_size, data, data_size);

  uint32_t crc = crc32(data_packet.messsage, (int)(header_size + data_size));

  memcpy(data_packet.messsage + header_size + data_size, &crc, crc_size);

  if (serial_port_->write(data_packet.messsage, data_packet_size) != static_cast<int32_t>(data_packet_size))
  {
      return 1;
  }

  return 0;
}

uint32_t sprinter::SerialCore::crc32(const bytePtr data, size_t length)
{
  uint32_t crc = 0xFFFFFFFF;

  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF];
  }

  return crc ^ 0xFFFFFFFF;
}