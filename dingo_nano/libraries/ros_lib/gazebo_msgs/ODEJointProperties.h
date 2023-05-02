#ifndef _ROS_gazebo_msgs_ODEJointProperties_h
#define _ROS_gazebo_msgs_ODEJointProperties_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

  class ODEJointProperties : public ros::Msg
  {
    public:
      uint32_t damping_length;
      typedef float _damping_type;
      _damping_type st_damping;
      _damping_type * damping;
      uint32_t hiStop_length;
      typedef float _hiStop_type;
      _hiStop_type st_hiStop;
      _hiStop_type * hiStop;
      uint32_t loStop_length;
      typedef float _loStop_type;
      _loStop_type st_loStop;
      _loStop_type * loStop;
      uint32_t erp_length;
      typedef float _erp_type;
      _erp_type st_erp;
      _erp_type * erp;
      uint32_t cfm_length;
      typedef float _cfm_type;
      _cfm_type st_cfm;
      _cfm_type * cfm;
      uint32_t stop_erp_length;
      typedef float _stop_erp_type;
      _stop_erp_type st_stop_erp;
      _stop_erp_type * stop_erp;
      uint32_t stop_cfm_length;
      typedef float _stop_cfm_type;
      _stop_cfm_type st_stop_cfm;
      _stop_cfm_type * stop_cfm;
      uint32_t fudge_factor_length;
      typedef float _fudge_factor_type;
      _fudge_factor_type st_fudge_factor;
      _fudge_factor_type * fudge_factor;
      uint32_t fmax_length;
      typedef float _fmax_type;
      _fmax_type st_fmax;
      _fmax_type * fmax;
      uint32_t vel_length;
      typedef float _vel_type;
      _vel_type st_vel;
      _vel_type * vel;

    ODEJointProperties():
      damping_length(0), st_damping(), damping(nullptr),
      hiStop_length(0), st_hiStop(), hiStop(nullptr),
      loStop_length(0), st_loStop(), loStop(nullptr),
      erp_length(0), st_erp(), erp(nullptr),
      cfm_length(0), st_cfm(), cfm(nullptr),
      stop_erp_length(0), st_stop_erp(), stop_erp(nullptr),
      stop_cfm_length(0), st_stop_cfm(), stop_cfm(nullptr),
      fudge_factor_length(0), st_fudge_factor(), fudge_factor(nullptr),
      fmax_length(0), st_fmax(), fmax(nullptr),
      vel_length(0), st_vel(), vel(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->damping_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->damping_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->damping_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->damping_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->damping_length);
      for( uint32_t i = 0; i < damping_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->damping[i]);
      }
      *(outbuffer + offset + 0) = (this->hiStop_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hiStop_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->hiStop_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->hiStop_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hiStop_length);
      for( uint32_t i = 0; i < hiStop_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->hiStop[i]);
      }
      *(outbuffer + offset + 0) = (this->loStop_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->loStop_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->loStop_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->loStop_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->loStop_length);
      for( uint32_t i = 0; i < loStop_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->loStop[i]);
      }
      *(outbuffer + offset + 0) = (this->erp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->erp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->erp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->erp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->erp_length);
      for( uint32_t i = 0; i < erp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->erp[i]);
      }
      *(outbuffer + offset + 0) = (this->cfm_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cfm_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cfm_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cfm_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cfm_length);
      for( uint32_t i = 0; i < cfm_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->cfm[i]);
      }
      *(outbuffer + offset + 0) = (this->stop_erp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stop_erp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stop_erp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stop_erp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_erp_length);
      for( uint32_t i = 0; i < stop_erp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stop_erp[i]);
      }
      *(outbuffer + offset + 0) = (this->stop_cfm_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stop_cfm_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stop_cfm_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stop_cfm_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_cfm_length);
      for( uint32_t i = 0; i < stop_cfm_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stop_cfm[i]);
      }
      *(outbuffer + offset + 0) = (this->fudge_factor_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fudge_factor_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fudge_factor_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fudge_factor_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fudge_factor_length);
      for( uint32_t i = 0; i < fudge_factor_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fudge_factor[i]);
      }
      *(outbuffer + offset + 0) = (this->fmax_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fmax_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fmax_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fmax_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fmax_length);
      for( uint32_t i = 0; i < fmax_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fmax[i]);
      }
      *(outbuffer + offset + 0) = (this->vel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_length);
      for( uint32_t i = 0; i < vel_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->vel[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t damping_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->damping_length);
      if(damping_lengthT > damping_length)
        this->damping = (float*)realloc(this->damping, damping_lengthT * sizeof(float));
      damping_length = damping_lengthT;
      for( uint32_t i = 0; i < damping_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_damping));
        memcpy( &(this->damping[i]), &(this->st_damping), sizeof(float));
      }
      uint32_t hiStop_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      hiStop_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      hiStop_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      hiStop_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->hiStop_length);
      if(hiStop_lengthT > hiStop_length)
        this->hiStop = (float*)realloc(this->hiStop, hiStop_lengthT * sizeof(float));
      hiStop_length = hiStop_lengthT;
      for( uint32_t i = 0; i < hiStop_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_hiStop));
        memcpy( &(this->hiStop[i]), &(this->st_hiStop), sizeof(float));
      }
      uint32_t loStop_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      loStop_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      loStop_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      loStop_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->loStop_length);
      if(loStop_lengthT > loStop_length)
        this->loStop = (float*)realloc(this->loStop, loStop_lengthT * sizeof(float));
      loStop_length = loStop_lengthT;
      for( uint32_t i = 0; i < loStop_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_loStop));
        memcpy( &(this->loStop[i]), &(this->st_loStop), sizeof(float));
      }
      uint32_t erp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      erp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      erp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      erp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->erp_length);
      if(erp_lengthT > erp_length)
        this->erp = (float*)realloc(this->erp, erp_lengthT * sizeof(float));
      erp_length = erp_lengthT;
      for( uint32_t i = 0; i < erp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_erp));
        memcpy( &(this->erp[i]), &(this->st_erp), sizeof(float));
      }
      uint32_t cfm_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cfm_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cfm_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cfm_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cfm_length);
      if(cfm_lengthT > cfm_length)
        this->cfm = (float*)realloc(this->cfm, cfm_lengthT * sizeof(float));
      cfm_length = cfm_lengthT;
      for( uint32_t i = 0; i < cfm_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_cfm));
        memcpy( &(this->cfm[i]), &(this->st_cfm), sizeof(float));
      }
      uint32_t stop_erp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      stop_erp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      stop_erp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      stop_erp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->stop_erp_length);
      if(stop_erp_lengthT > stop_erp_length)
        this->stop_erp = (float*)realloc(this->stop_erp, stop_erp_lengthT * sizeof(float));
      stop_erp_length = stop_erp_lengthT;
      for( uint32_t i = 0; i < stop_erp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_stop_erp));
        memcpy( &(this->stop_erp[i]), &(this->st_stop_erp), sizeof(float));
      }
      uint32_t stop_cfm_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      stop_cfm_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      stop_cfm_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      stop_cfm_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->stop_cfm_length);
      if(stop_cfm_lengthT > stop_cfm_length)
        this->stop_cfm = (float*)realloc(this->stop_cfm, stop_cfm_lengthT * sizeof(float));
      stop_cfm_length = stop_cfm_lengthT;
      for( uint32_t i = 0; i < stop_cfm_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_stop_cfm));
        memcpy( &(this->stop_cfm[i]), &(this->st_stop_cfm), sizeof(float));
      }
      uint32_t fudge_factor_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fudge_factor_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fudge_factor_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fudge_factor_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fudge_factor_length);
      if(fudge_factor_lengthT > fudge_factor_length)
        this->fudge_factor = (float*)realloc(this->fudge_factor, fudge_factor_lengthT * sizeof(float));
      fudge_factor_length = fudge_factor_lengthT;
      for( uint32_t i = 0; i < fudge_factor_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_fudge_factor));
        memcpy( &(this->fudge_factor[i]), &(this->st_fudge_factor), sizeof(float));
      }
      uint32_t fmax_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fmax_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fmax_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fmax_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fmax_length);
      if(fmax_lengthT > fmax_length)
        this->fmax = (float*)realloc(this->fmax, fmax_lengthT * sizeof(float));
      fmax_length = fmax_lengthT;
      for( uint32_t i = 0; i < fmax_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_fmax));
        memcpy( &(this->fmax[i]), &(this->st_fmax), sizeof(float));
      }
      uint32_t vel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vel_length);
      if(vel_lengthT > vel_length)
        this->vel = (float*)realloc(this->vel, vel_lengthT * sizeof(float));
      vel_length = vel_lengthT;
      for( uint32_t i = 0; i < vel_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_vel));
        memcpy( &(this->vel[i]), &(this->st_vel), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "gazebo_msgs/ODEJointProperties"; };
    virtual const char * getMD5() override { return "1b744c32a920af979f53afe2f9c3511f"; };

  };

}
#endif
