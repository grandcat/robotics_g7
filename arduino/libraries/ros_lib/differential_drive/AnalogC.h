#ifndef _ROS_differential_drive_AnalogC_h
#define _ROS_differential_drive_AnalogC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class AnalogC : public ros::Msg
  {
    public:
      uint16_t ch1;
      uint16_t ch2;
      uint16_t ch3;
      uint16_t ch4;
      uint16_t ch5;
      uint16_t ch6;
      uint16_t ch7;
      uint16_t ch8;
      uint8_t on_batt;
      float cell1;
      float cell2;
      float cell3;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ch1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch1);
      *(outbuffer + offset + 0) = (this->ch2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch2);
      *(outbuffer + offset + 0) = (this->ch3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch3);
      *(outbuffer + offset + 0) = (this->ch4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch4);
      *(outbuffer + offset + 0) = (this->ch5 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch5 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch5);
      *(outbuffer + offset + 0) = (this->ch6 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch6 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch6);
      *(outbuffer + offset + 0) = (this->ch7 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch7 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch7);
      *(outbuffer + offset + 0) = (this->ch8 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ch8 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch8);
      *(outbuffer + offset + 0) = (this->on_batt >> (8 * 0)) & 0xFF;
      offset += sizeof(this->on_batt);
      union {
        float real;
        uint32_t base;
      } u_cell1;
      u_cell1.real = this->cell1;
      *(outbuffer + offset + 0) = (u_cell1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell1);
      union {
        float real;
        uint32_t base;
      } u_cell2;
      u_cell2.real = this->cell2;
      *(outbuffer + offset + 0) = (u_cell2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell2);
      union {
        float real;
        uint32_t base;
      } u_cell3;
      u_cell3.real = this->cell3;
      *(outbuffer + offset + 0) = (u_cell3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->ch1 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch1);
      this->ch2 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch2);
      this->ch3 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch3);
      this->ch4 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch4 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch4);
      this->ch5 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch5 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch5);
      this->ch6 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch6 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch6);
      this->ch7 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch7 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch7);
      this->ch8 =  ((uint16_t) (*(inbuffer + offset)));
      this->ch8 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ch8);
      this->on_batt =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->on_batt);
      union {
        float real;
        uint32_t base;
      } u_cell1;
      u_cell1.base = 0;
      u_cell1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell1 = u_cell1.real;
      offset += sizeof(this->cell1);
      union {
        float real;
        uint32_t base;
      } u_cell2;
      u_cell2.base = 0;
      u_cell2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell2 = u_cell2.real;
      offset += sizeof(this->cell2);
      union {
        float real;
        uint32_t base;
      } u_cell3;
      u_cell3.base = 0;
      u_cell3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell3 = u_cell3.real;
      offset += sizeof(this->cell3);
     return offset;
    }

    const char * getType(){ return "differential_drive/AnalogC"; };
    const char * getMD5(){ return "2e0957cb939ed78036365a4d8012012f"; };

  };

}
#endif