#ifndef _ROS_differential_drive_KeyEvent_h
#define _ROS_differential_drive_KeyEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class KeyEvent : public ros::Msg
  {
    public:
      float timestamp;
      uint16_t sym;
      uint8_t pressed;
      char * name;

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
      *(outbuffer + offset + 0) = (this->sym >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sym >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sym);
      *(outbuffer + offset + 0) = (this->pressed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pressed);
      uint32_t * length_name = (uint32_t *)(outbuffer + offset);
      *length_name = strlen( (const char*) this->name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, *length_name);
      offset += *length_name;
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
      this->sym =  ((uint16_t) (*(inbuffer + offset)));
      this->sym |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sym);
      this->pressed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pressed);
      uint32_t length_name = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return "differential_drive/KeyEvent"; };
    const char * getMD5(){ return "644813ba9e39b249cc8e56e2fa499967"; };

  };

}
#endif