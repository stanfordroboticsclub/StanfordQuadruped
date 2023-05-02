#ifndef _ROS_trajectory_msgs_MultiDOFJointTrajectoryPoint_h
#define _ROS_trajectory_msgs_MultiDOFJointTrajectoryPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "ros/duration.h"

namespace trajectory_msgs
{

  class MultiDOFJointTrajectoryPoint : public ros::Msg
  {
    public:
      uint32_t transforms_length;
      typedef geometry_msgs::Transform _transforms_type;
      _transforms_type st_transforms;
      _transforms_type * transforms;
      uint32_t velocities_length;
      typedef geometry_msgs::Twist _velocities_type;
      _velocities_type st_velocities;
      _velocities_type * velocities;
      uint32_t accelerations_length;
      typedef geometry_msgs::Twist _accelerations_type;
      _accelerations_type st_accelerations;
      _accelerations_type * accelerations;
      typedef ros::Duration _time_from_start_type;
      _time_from_start_type time_from_start;

    MultiDOFJointTrajectoryPoint():
      transforms_length(0), st_transforms(), transforms(nullptr),
      velocities_length(0), st_velocities(), velocities(nullptr),
      accelerations_length(0), st_accelerations(), accelerations(nullptr),
      time_from_start()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->transforms_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transforms_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transforms_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transforms_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transforms_length);
      for( uint32_t i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_length);
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += this->velocities[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->accelerations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accelerations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accelerations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accelerations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelerations_length);
      for( uint32_t i = 0; i < accelerations_length; i++){
      offset += this->accelerations[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->time_from_start.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.sec);
      *(outbuffer + offset + 0) = (this->time_from_start.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t transforms_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      transforms_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->transforms_length);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::Transform*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::Transform));
      transforms_length = transforms_lengthT;
      for( uint32_t i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::Transform));
      }
      uint32_t velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_length);
      if(velocities_lengthT > velocities_length)
        this->velocities = (geometry_msgs::Twist*)realloc(this->velocities, velocities_lengthT * sizeof(geometry_msgs::Twist));
      velocities_length = velocities_lengthT;
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += this->st_velocities.deserialize(inbuffer + offset);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(geometry_msgs::Twist));
      }
      uint32_t accelerations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      accelerations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      accelerations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      accelerations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->accelerations_length);
      if(accelerations_lengthT > accelerations_length)
        this->accelerations = (geometry_msgs::Twist*)realloc(this->accelerations, accelerations_lengthT * sizeof(geometry_msgs::Twist));
      accelerations_length = accelerations_lengthT;
      for( uint32_t i = 0; i < accelerations_length; i++){
      offset += this->st_accelerations.deserialize(inbuffer + offset);
        memcpy( &(this->accelerations[i]), &(this->st_accelerations), sizeof(geometry_msgs::Twist));
      }
      this->time_from_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.sec);
      this->time_from_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.nsec);
     return offset;
    }

    virtual const char * getType() override { return "trajectory_msgs/MultiDOFJointTrajectoryPoint"; };
    virtual const char * getMD5() override { return "3ebe08d1abd5b65862d50e09430db776"; };

  };

}
#endif
