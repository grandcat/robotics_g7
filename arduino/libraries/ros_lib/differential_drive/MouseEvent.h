#ifndef _ROS_differential_drive_MouseEvent_h
#define _ROS_differential_drive_MouseEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class MouseEvent : public ros::Msg
  {
    public:
      float timestamp;
      int16_t dx;
      int16_t dy;
      uint8_t button;
      uint8_t pressed;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_timestamp = (long *) &(this->timestamp);
      int32_t exp_timestamp = (((*val_timestamp)>>23)&255);
      if(exp_timestamp != 0)
        exp_timestamp += 1023-127;
      int32_t sig_timestamp = *val_timestamp;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_timestamp<<5) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp>>3) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_timestamp<<4) & 0xF0) | ((sig_timestamp>>19)&0x0F);
      *(outbuffer + offset++) = (exp_timestamp>>4) & 0x7F;
      if(this->timestamp < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        int16_t real;
        uint16_t base;
      } u_dx;
      u_dx.real = this->dx;
      *(outbuffer + offset + 0) = (u_dx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dx.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dx);
      union {
        int16_t real;
        uint16_t base;
      } u_dy;
      u_dy.real = this->dy;
      *(outbuffer + offset + 0) = (u_dy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dy.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dy);
      *(outbuffer + offset + 0) = (this->button >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button);
      *(outbuffer + offset + 0) = (this->pressed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pressed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_timestamp = (uint32_t*) &(this->timestamp);
      offset += 3;
      *val_timestamp = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_timestamp = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_timestamp |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_timestamp !=0)
        *val_timestamp |= ((exp_timestamp)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->timestamp = -this->timestamp;
      union {
        int16_t real;
        uint16_t base;
      } u_dx;
      u_dx.base = 0;
      u_dx.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dx.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dx = u_dx.real;
      offset += sizeof(this->dx);
      union {
        int16_t real;
        uint16_t base;
      } u_dy;
      u_dy.base = 0;
      u_dy.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dy.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dy = u_dy.real;
      offset += sizeof(this->dy);
      this->button =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->button);
      this->pressed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pressed);
     return offset;
    }

    const char * getType(){ return "differential_drive/MouseEvent"; };
    const char * getMD5(){ return "ef8c9f3ac43b76bcf800e7d5001209fb"; };

  };

}
#endif