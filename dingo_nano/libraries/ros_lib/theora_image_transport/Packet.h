#ifndef _ROS_theora_image_transport_Packet_h
#define _ROS_theora_image_transport_Packet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace theora_image_transport
{

  class Packet : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;
      typedef int32_t _b_o_s_type;
      _b_o_s_type b_o_s;
      typedef int32_t _e_o_s_type;
      _e_o_s_type e_o_s;
      typedef int64_t _granulepos_type;
      _granulepos_type granulepos;
      typedef int64_t _packetno_type;
      _packetno_type packetno;

    Packet():
      header(),
      data_length(0), st_data(), data(nullptr),
      b_o_s(0),
      e_o_s(0),
      granulepos(0),
      packetno(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_b_o_s;
      u_b_o_s.real = this->b_o_s;
      *(outbuffer + offset + 0) = (u_b_o_s.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b_o_s.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_b_o_s.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_b_o_s.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b_o_s);
      union {
        int32_t real;
        uint32_t base;
      } u_e_o_s;
      u_e_o_s.real = this->e_o_s;
      *(outbuffer + offset + 0) = (u_e_o_s.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_e_o_s.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_e_o_s.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_e_o_s.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e_o_s);
      union {
        int64_t real;
        uint64_t base;
      } u_granulepos;
      u_granulepos.real = this->granulepos;
      *(outbuffer + offset + 0) = (u_granulepos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_granulepos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_granulepos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_granulepos.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_granulepos.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_granulepos.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_granulepos.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_granulepos.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->granulepos);
      union {
        int64_t real;
        uint64_t base;
      } u_packetno;
      u_packetno.real = this->packetno;
      *(outbuffer + offset + 0) = (u_packetno.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_packetno.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_packetno.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_packetno.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_packetno.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_packetno.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_packetno.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_packetno.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->packetno);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint8_t*)realloc(this->data, data_lengthT * sizeof(uint8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint8_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_b_o_s;
      u_b_o_s.base = 0;
      u_b_o_s.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b_o_s.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_b_o_s.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_b_o_s.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->b_o_s = u_b_o_s.real;
      offset += sizeof(this->b_o_s);
      union {
        int32_t real;
        uint32_t base;
      } u_e_o_s;
      u_e_o_s.base = 0;
      u_e_o_s.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_e_o_s.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_e_o_s.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_e_o_s.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->e_o_s = u_e_o_s.real;
      offset += sizeof(this->e_o_s);
      union {
        int64_t real;
        uint64_t base;
      } u_granulepos;
      u_granulepos.base = 0;
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_granulepos.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->granulepos = u_granulepos.real;
      offset += sizeof(this->granulepos);
      union {
        int64_t real;
        uint64_t base;
      } u_packetno;
      u_packetno.base = 0;
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_packetno.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->packetno = u_packetno.real;
      offset += sizeof(this->packetno);
     return offset;
    }

    virtual const char * getType() override { return "theora_image_transport/Packet"; };
    virtual const char * getMD5() override { return "33ac4e14a7cff32e7e0d65f18bb410f3"; };

  };

}
#endif
