#ifndef _ROS_differential_drive_Speed_h
#define _ROS_differential_drive_Speed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace differential_drive
{

  class Speed : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float W1;
      float W2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_W1;
      u_W1.real = this->W1;
      *(outbuffer + offset + 0) = (u_W1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_W1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_W1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_W1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->W1);
      union {
        float real;
        uint32_t base;
      } u_W2;
      u_W2.real = this->W2;
      *(outbuffer + offset + 0) = (u_W2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_W2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_W2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_W2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->W2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_W1;
      u_W1.base = 0;
      u_W1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_W1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_W1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_W1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->W1 = u_W1.real;
      offset += sizeof(this->W1);
      union {
        float real;
        uint32_t base;
      } u_W2;
      u_W2.base = 0;
      u_W2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_W2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_W2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_W2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->W2 = u_W2.real;
      offset += sizeof(this->W2);
     return offset;
    }

    const char * getType(){ return "differential_drive/Speed"; };
    const char * getMD5(){ return "ae0c4a7348b1a8bfa79b856330d95cad"; };

  };

}
#endif