#ifndef _ROS_differential_drive_Params_h
#define _ROS_differential_drive_Params_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class Params : public ros::Msg
  {
    public:
      uint16_t K;
      uint16_t KI;
      uint16_t INT_MAX;
      uint16_t ticks;
      float r;
      float r_l;
      float r_r;
      float B;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->K >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->K >> (8 * 1)) & 0xFF;
      offset += sizeof(this->K);
      *(outbuffer + offset + 0) = (this->KI >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->KI >> (8 * 1)) & 0xFF;
      offset += sizeof(this->KI);
      *(outbuffer + offset + 0) = (this->INT_MAX >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->INT_MAX >> (8 * 1)) & 0xFF;
      offset += sizeof(this->INT_MAX);
      *(outbuffer + offset + 0) = (this->ticks >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ticks >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticks);
      union {
        float real;
        uint32_t base;
      } u_r;
      u_r.real = this->r;
      *(outbuffer + offset + 0) = (u_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r);
      union {
        float real;
        uint32_t base;
      } u_r_l;
      u_r_l.real = this->r_l;
      *(outbuffer + offset + 0) = (u_r_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_l.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_l.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_l.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_l);
      union {
        float real;
        uint32_t base;
      } u_r_r;
      u_r_r.real = this->r_r;
      *(outbuffer + offset + 0) = (u_r_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_r);
      union {
        float real;
        uint32_t base;
      } u_B;
      u_B.real = this->B;
      *(outbuffer + offset + 0) = (u_B.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_B.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_B.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_B.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->B);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->K =  ((uint16_t) (*(inbuffer + offset)));
      this->K |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->K);
      this->KI =  ((uint16_t) (*(inbuffer + offset)));
      this->KI |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->KI);
      this->INT_MAX =  ((uint16_t) (*(inbuffer + offset)));
      this->INT_MAX |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->INT_MAX);
      this->ticks =  ((uint16_t) (*(inbuffer + offset)));
      this->ticks |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ticks);
      union {
        float real;
        uint32_t base;
      } u_r;
      u_r.base = 0;
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r = u_r.real;
      offset += sizeof(this->r);
      union {
        float real;
        uint32_t base;
      } u_r_l;
      u_r_l.base = 0;
      u_r_l.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_l.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_l.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_l.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_l = u_r_l.real;
      offset += sizeof(this->r_l);
      union {
        float real;
        uint32_t base;
      } u_r_r;
      u_r_r.base = 0;
      u_r_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_r = u_r_r.real;
      offset += sizeof(this->r_r);
      union {
        float real;
        uint32_t base;
      } u_B;
      u_B.base = 0;
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->B = u_B.real;
      offset += sizeof(this->B);
     return offset;
    }

    const char * getType(){ return "differential_drive/Params"; };
    const char * getMD5(){ return "8950f824993d2867219a00186d4620f7"; };

  };

}
#endif