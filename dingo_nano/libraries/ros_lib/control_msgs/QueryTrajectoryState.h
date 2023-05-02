#ifndef _ROS_SERVICE_QueryTrajectoryState_h
#define _ROS_SERVICE_QueryTrajectoryState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace control_msgs
{

static const char QUERYTRAJECTORYSTATE[] = "control_msgs/QueryTrajectoryState";

  class QueryTrajectoryStateRequest : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;

    QueryTrajectoryStateRequest():
      time()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
     return offset;
    }

    virtual const char * getType() override { return QUERYTRAJECTORYSTATE; };
    virtual const char * getMD5() override { return "556a4fb76023a469987922359d08a844"; };

  };

  class QueryTrajectoryStateResponse : public ros::Msg
  {
    public:
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t position_length;
      typedef float _position_type;
      _position_type st_position;
      _position_type * position;
      uint32_t velocity_length;
      typedef float _velocity_type;
      _velocity_type st_velocity;
      _velocity_type * velocity;
      uint32_t acceleration_length;
      typedef float _acceleration_type;
      _acceleration_type st_acceleration;
      _acceleration_type * acceleration;

    QueryTrajectoryStateResponse():
      name_length(0), st_name(), name(nullptr),
      position_length(0), st_position(), position(nullptr),
      velocity_length(0), st_velocity(), velocity(nullptr),
      acceleration_length(0), st_acceleration(), acceleration(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_length);
      for( uint32_t i = 0; i < position_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position[i]);
      }
      *(outbuffer + offset + 0) = (this->velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_length);
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->acceleration_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->acceleration_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->acceleration_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->acceleration_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acceleration_length);
      for( uint32_t i = 0; i < acceleration_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_length);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      position_length = position_lengthT;
      for( uint32_t i = 0; i < position_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position));
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
      uint32_t velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_length);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      velocity_length = velocity_lengthT;
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocity));
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
      }
      uint32_t acceleration_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->acceleration_length);
      if(acceleration_lengthT > acceleration_length)
        this->acceleration = (float*)realloc(this->acceleration, acceleration_lengthT * sizeof(float));
      acceleration_length = acceleration_lengthT;
      for( uint32_t i = 0; i < acceleration_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_acceleration));
        memcpy( &(this->acceleration[i]), &(this->st_acceleration), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return QUERYTRAJECTORYSTATE; };
    virtual const char * getMD5() override { return "1f1a6554ad060f44d013e71868403c1a"; };

  };

  class QueryTrajectoryState {
    public:
    typedef QueryTrajectoryStateRequest Request;
    typedef QueryTrajectoryStateResponse Response;
  };

}
#endif
