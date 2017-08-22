#include "fineproto.h"

void fineproto_init()
{

}

void parse_all_messages()
{

}

void parse_message(FineMessage message)
{

}

void _continous_setup()
{

}

void _continous_advance()
{
  // iterate over callbacks until a connected 
  while(_fineproto.get_data_for[_fineproto.pri] == NULL)
  {
    _fineproto.pri++;
    _fineproto.pri &= 0xF;
  }
  _fineproto.to_send = _create_data_message(_fineproto.pri);
  _fineproto.pri++;
  _fineproto.pri &= 0xF;
  // TODO: send the message
}

void _got_message()
{
  _fineproto.rcv_queue[_fineproto.rci] = _fineproto.last_rcv;
  // TODO: enable DMA here again
  _fineproto.rci++;
  _fineproto.rci %= QUEUE_SIZE;
}

inline uint8_t _calculate_checksum(FineMessage msg)
{
  return (msg.header + msg.command + msg.data[0] + msg.data[1]) & 0xFF;
}

FineMessage _create_data_message(Sensor sensor)
{
  FineMessage fm_to_ret;
  fm_to_ret.header = 0xF1;
  fm_to_ret.command = 0x02 | (((uint8_t)sensor) << 4);
  uint16_t sensor_data = _fineproto.get_data_for[sensor]();
  fm_to_ret.data[0] = sensor_data >> 8;
  fm_to_ret.data[1] = sensor_data & 0xFF;
  fm_to_ret.checksum = _calculate_checksum(fm_to_ret);
}
