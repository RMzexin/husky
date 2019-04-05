#ifndef _ROS_imca_msgs_fire_h
#define _ROS_imca_msgs_fire_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace imca_msgs
{

  class fire : public ros::Msg
  {
    public:
      typedef int8_t _moca_type;
      _moca_type moca;
      typedef int32_t _bodan_type;
      _bodan_type bodan;

    fire():
      moca(0),
      bodan(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_moca;
      u_moca.real = this->moca;
      *(outbuffer + offset + 0) = (u_moca.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->moca);
      union {
        int32_t real;
        uint32_t base;
      } u_bodan;
      u_bodan.real = this->bodan;
      *(outbuffer + offset + 0) = (u_bodan.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bodan.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bodan.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bodan.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bodan);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_moca;
      u_moca.base = 0;
      u_moca.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->moca = u_moca.real;
      offset += sizeof(this->moca);
      union {
        int32_t real;
        uint32_t base;
      } u_bodan;
      u_bodan.base = 0;
      u_bodan.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bodan.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bodan.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bodan.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bodan = u_bodan.real;
      offset += sizeof(this->bodan);
     return offset;
    }

    const char * getType(){ return "imca_msgs/fire"; };
    const char * getMD5(){ return "ba82df5fd1dc0c30224797aeb56790c7"; };

  };

}
#endif
