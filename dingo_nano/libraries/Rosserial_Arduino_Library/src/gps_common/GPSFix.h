#ifndef _ROS_gps_common_GPSFix_h
#define _ROS_gps_common_GPSFix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "gps_common/GPSStatus.h"

namespace gps_common
{

  class GPSFix : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef gps_common::GPSStatus _status_type;
      _status_type status;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef float _track_type;
      _track_type track;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _climb_type;
      _climb_type climb;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _dip_type;
      _dip_type dip;
      typedef float _time_type;
      _time_type time;
      typedef float _gdop_type;
      _gdop_type gdop;
      typedef float _pdop_type;
      _pdop_type pdop;
      typedef float _hdop_type;
      _hdop_type hdop;
      typedef float _vdop_type;
      _vdop_type vdop;
      typedef float _tdop_type;
      _tdop_type tdop;
      typedef float _err_type;
      _err_type err;
      typedef float _err_horz_type;
      _err_horz_type err_horz;
      typedef float _err_vert_type;
      _err_vert_type err_vert;
      typedef float _err_track_type;
      _err_track_type err_track;
      typedef float _err_speed_type;
      _err_speed_type err_speed;
      typedef float _err_climb_type;
      _err_climb_type err_climb;
      typedef float _err_time_type;
      _err_time_type err_time;
      typedef float _err_pitch_type;
      _err_pitch_type err_pitch;
      typedef float _err_roll_type;
      _err_roll_type err_roll;
      typedef float _err_dip_type;
      _err_dip_type err_dip;
      float position_covariance[9];
      typedef uint8_t _position_covariance_type_type;
      _position_covariance_type_type position_covariance_type;
      enum { COVARIANCE_TYPE_UNKNOWN =  0 };
      enum { COVARIANCE_TYPE_APPROXIMATED =  1 };
      enum { COVARIANCE_TYPE_DIAGONAL_KNOWN =  2 };
      enum { COVARIANCE_TYPE_KNOWN =  3 };

    GPSFix():
      header(),
      status(),
      latitude(0),
      longitude(0),
      altitude(0),
      track(0),
      speed(0),
      climb(0),
      pitch(0),
      roll(0),
      dip(0),
      time(0),
      gdop(0),
      pdop(0),
      hdop(0),
      vdop(0),
      tdop(0),
      err(0),
      err_horz(0),
      err_vert(0),
      err_track(0),
      err_speed(0),
      err_climb(0),
      err_time(0),
      err_pitch(0),
      err_roll(0),
      err_dip(0),
      position_covariance(),
      position_covariance_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->track);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->climb);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->dip);
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      offset += serializeAvrFloat64(outbuffer + offset, this->gdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->pdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->hdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->vdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->tdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->err);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_horz);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_vert);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_track);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_climb);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_dip);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_covariance[i]);
      }
      *(outbuffer + offset + 0) = (this->position_covariance_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->track));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->climb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dip));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_horz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_vert));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_track));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_climb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_dip));
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position_covariance[i]));
      }
      this->position_covariance_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_covariance_type);
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("gps_common/GPSFix");};
    #else
        const char * getType() { return  PSTR("gps_common/GPSFix");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("3db3d0a7bc53054c67c528af84710b70");};
    #else
        const char * getMD5() { return  PSTR("3db3d0a7bc53054c67c528af84710b70");};
    #endif

  };

}
#endif
