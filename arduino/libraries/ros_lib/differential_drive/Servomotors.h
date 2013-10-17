#ifndef _ROS_differential_drive_Servomotors_h
#define _ROS_differential_drive_Servomotors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class Servomotors : public ros::Msg
  {
    public:
      uint8_t servo1;
      uint8_t servoangle[8];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->servo1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->servo1);
      unsigned char * servoangle_val = (unsigned char *) this->servoangle;
      for( uint8_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->servoangle[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->servoangle[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->servo1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->servo1);
      uint8_t * servoangle_val = (uint8_t*) this->servoangle;
      for( uint8_t i = 0; i < 8; i++){
      this->servoangle[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->servoangle[i]);
      }
     return offset;
    }

    const char * getType(){ return "differential_drive/Servomotors"; };
    const char * getMD5(){ return "f51d948c20e299a516b60c1bba9b447f"; };

  };

}
#endif