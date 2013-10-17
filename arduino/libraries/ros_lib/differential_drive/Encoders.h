#ifndef _ROS_differential_drive_Encoders_h
#define _ROS_differential_drive_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class Encoders : public ros::Msg
  {
    public:
      int32_t encoder1;
      int32_t encoder2;
      int32_t delta_encoder1;
      int32_t delta_encoder2;
      int32_t timestamp;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_encoder1;
      u_encoder1.real = this->encoder1;
      *(outbuffer + offset + 0) = (u_encoder1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder1);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder2;
      u_encoder2.real = this->encoder2;
      *(outbuffer + offset + 0) = (u_encoder2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder2);
      union {
        int32_t real;
        uint32_t base;
      } u_delta_encoder1;
      u_delta_encoder1.real = this->delta_encoder1;
      *(outbuffer + offset + 0) = (u_delta_encoder1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta_encoder1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta_encoder1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta_encoder1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta_encoder1);
      union {
        int32_t real;
        uint32_t base;
      } u_delta_encoder2;
      u_delta_encoder2.real = this->delta_encoder2;
      *(outbuffer + offset + 0) = (u_delta_encoder2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta_encoder2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta_encoder2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta_encoder2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta_encoder2);
      union {
        int32_t real;
        uint32_t base;
      } u_timestamp;
      u_timestamp.real = this->timestamp;
      *(outbuffer + offset + 0) = (u_timestamp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timestamp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timestamp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timestamp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_encoder1;
      u_encoder1.base = 0;
      u_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder1 = u_encoder1.real;
      offset += sizeof(this->encoder1);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder2;
      u_encoder2.base = 0;
      u_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder2 = u_encoder2.real;
      offset += sizeof(this->encoder2);
      union {
        int32_t real;
        uint32_t base;
      } u_delta_encoder1;
      u_delta_encoder1.base = 0;
      u_delta_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta_encoder1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta_encoder1 = u_delta_encoder1.real;
      offset += sizeof(this->delta_encoder1);
      union {
        int32_t real;
        uint32_t base;
      } u_delta_encoder2;
      u_delta_encoder2.base = 0;
      u_delta_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta_encoder2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta_encoder2 = u_delta_encoder2.real;
      offset += sizeof(this->delta_encoder2);
      union {
        int32_t real;
        uint32_t base;
      } u_timestamp;
      u_timestamp.base = 0;
      u_timestamp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timestamp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timestamp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timestamp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timestamp = u_timestamp.real;
      offset += sizeof(this->timestamp);
     return offset;
    }

    const char * getType(){ return "differential_drive/Encoders"; };
    const char * getMD5(){ return "6363193e7f66cd2bf0e60c5eb25e1bd5"; };

  };

}
#endif