#ifndef _ROS_differential_drive_PWM_h
#define _ROS_differential_drive_PWM_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace differential_drive
{

  class PWM : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int32_t PWM1;
      int32_t PWM2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_PWM1;
      u_PWM1.real = this->PWM1;
      *(outbuffer + offset + 0) = (u_PWM1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_PWM1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_PWM1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_PWM1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->PWM1);
      union {
        int32_t real;
        uint32_t base;
      } u_PWM2;
      u_PWM2.real = this->PWM2;
      *(outbuffer + offset + 0) = (u_PWM2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_PWM2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_PWM2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_PWM2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->PWM2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_PWM1;
      u_PWM1.base = 0;
      u_PWM1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_PWM1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_PWM1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_PWM1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->PWM1 = u_PWM1.real;
      offset += sizeof(this->PWM1);
      union {
        int32_t real;
        uint32_t base;
      } u_PWM2;
      u_PWM2.base = 0;
      u_PWM2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_PWM2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_PWM2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_PWM2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->PWM2 = u_PWM2.real;
      offset += sizeof(this->PWM2);
     return offset;
    }

    const char * getType(){ return "differential_drive/PWM"; };
    const char * getMD5(){ return "7536f824b238f3bc8ae162f5a97c5bfc"; };

  };

}
#endif