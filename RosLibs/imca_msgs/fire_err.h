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
      typedef const char* _moca_type;
      _moca_type moca;
      typedef int32_t _bodan_type;
      _bodan_type bodan;

    fire():
      moca(""),
      bodan(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_moca = strlen(this->moca);
      varToArr(outbuffer + offset, length_moca);
      offset += 4;
      memcpy(outbuffer + offset, this->moca, length_moca);
      offset += length_moca;
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
      uint32_t length_moca;
      arrToVar(length_moca, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_moca; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_moca-1]=0;
      this->moca = (char *)(inbuffer + offset-1);
      offset += length_moca;
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
    const char * getMD5(){ return "00dd7ead202fef78f3a7f7529fa9ddf9"; };

  };

}
#endif
