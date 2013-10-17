#ifndef _ROS_differential_drive_Sharp_h
#define _ROS_differential_drive_Sharp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class Sharp : public ros::Msg
  {
    public:
      uint16_t u0;
      uint16_t u1;
      uint16_t u2;
      uint16_t u3;
      uint16_t u4;
      uint16_t u5;
      uint16_t u6;
      uint16_t u7;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->u0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u0);
      *(outbuffer + offset + 0) = (this->u1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u1);
      *(outbuffer + offset + 0) = (this->u2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u2);
      *(outbuffer + offset + 0) = (this->u3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u3);
      *(outbuffer + offset + 0) = (this->u4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u4);
      *(outbuffer + offset + 0) = (this->u5 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u5 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u5);
      *(outbuffer + offset + 0) = (this->u6 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u6 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u6);
      *(outbuffer + offset + 0) = (this->u7 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->u7 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->u7);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->u0 =  ((uint16_t) (*(inbuffer + offset)));
      this->u0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u0);
      this->u1 =  ((uint16_t) (*(inbuffer + offset)));
      this->u1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u1);
      this->u2 =  ((uint16_t) (*(inbuffer + offset)));
      this->u2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u2);
      this->u3 =  ((uint16_t) (*(inbuffer + offset)));
      this->u3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u3);
      this->u4 =  ((uint16_t) (*(inbuffer + offset)));
      this->u4 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u4);
      this->u5 =  ((uint16_t) (*(inbuffer + offset)));
      this->u5 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u5);
      this->u6 =  ((uint16_t) (*(inbuffer + offset)));
      this->u6 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u6);
      this->u7 =  ((uint16_t) (*(inbuffer + offset)));
      this->u7 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->u7);
     return offset;
    }

    const char * getType(){ return "differential_drive/Sharp"; };
    const char * getMD5(){ return "9e17fb25fb2f8b8cf0b60887d5e8cc66"; };

  };

}
#endif