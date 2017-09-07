#include "fineproto.h"

void fineproto_add_sensor(uint16_t (*get_data_for)(void))
{
  //TODO: literally just connect the dots
}

void fineproto_init()
{
  memset(&_fineproto, 0, sizeof(FineProtocol));
  _fineproto.rdi = QUEUE_SIZE-1;
  //TODO: send >>AT+START\r\n<< to bt
  //and start DMA
}

void fp_parse_all_messages()
{
  while(_fineproto.rdi != _fineproto.rci)
  {
    fp_parse_message(_fineproto.rcv_queue[_fineproto.rdi]);
    _fineproto.rdi++;
    _fineproto.rdi %= QUEUE_SIZE;
  }
}

void fp_parse_message(FineMessage message)
{
  if(message.header != 0xF2 || message.checksum != _calculate_checksum(message))
    return;
  
  FineMessage returned;
  
  if(message.command & 0x0F == QUERY_SENSOR)
  {
    returned = _fp_create_data_message(message.command>>4);
    //todo: get data, send data
  }
  else{
    returned.header = FINEPROTO_HEADER;
    returned.command = message.command;
    switch(message.command)
    {
      case QUERY_CAPABILITIES:
        //TODO
        // iterate over func pointers, if !=0 put 1, then >>1 etc.
        break;
      case QUERY_FWVER:
        returned.data[0] = BUILDDAY;
        returned.data[1] = BUILDMONTH;
        break;
      case QUERY_CONTINOUS:
        _fineproto.continuous_timer = message.data[0]<<8 | message.data[1];
        _fp_continuous_setup();
        //TODO :^)
        // no reply here!
        return;
      case QUERY_HANDSHAKE:
        //TODO
        returned.data[0] = message.data[0] + 0xF1;
        returned.data[1] = message.data[1] + 0xF1;
        break;
      default:
        //unknown command, no reply.
        break;
    }
    returned.checksum = _calculate_checksum(returned);
  }
}

void _fp_continuous_setup()
{
  if(_fineproto.continuous_timer == 0)
  {
    _fp_continuous_stop();
    return;
  }
  //1. take the timer value from _fineproto
  //2. multiply by timer cycles for 1 sec
  //3. start timer 
}

void _fp_continuous_stop()
{
  //1. stop the presses!
}

void _fp_continuous_advance()
{
  // iterate over callbacks until a configured sensor comes up 
  while(_fineproto.get_data_for[_fineproto.pri] == 0)
  {
    _fineproto.pri++;
    _fineproto.pri &= 0xF;
  }
  _fineproto.to_send = _create_data_message(_fineproto.pri);
  _fineproto.pri++;
  _fineproto.pri &= 0xF;
  // TODO: send the message
}

void _fp_got_message()
{
  _fineproto.rcv_queue[_fineproto.rci] = _fineproto.last_rcv;
  // TODO: enable DMA here again
  _fineproto.rci++;
  _fineproto.rci %= QUEUE_SIZE;
}

inline uint8_t _fp_calculate_checksum(FineMessage msg)
{
  return (msg.header + msg.command + msg.data[0] + msg.data[1]) & 0xFF;
}

FineMessage _fp_create_data_message(Sensor sensor)
{
  FineMessage fm_to_ret;
  fm_to_ret.header = 0xF1;
  fm_to_ret.command = 0x02 | (((uint8_t)sensor) << 4);
  uint16_t sensor_data = _fineproto.get_data_for[sensor]();
  fm_to_ret.data[0] = sensor_data >> 8;
  fm_to_ret.data[1] = sensor_data & 0xFF;
  fm_to_ret.checksum = _fp_calculate_checksum(fm_to_ret);
  return fm_to_ret;
}
