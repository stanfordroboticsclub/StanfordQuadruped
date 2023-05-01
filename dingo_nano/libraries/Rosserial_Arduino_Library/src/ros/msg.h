/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ROS_MSG_H_
#define _ROS_MSG_H_

#include <stdint.h>
#include <stddef.h>
#include <cstring>

namespace ros
{

/* Base Message Type */
class Msg
{
public:
  virtual int serialize(unsigned char *outbuffer) const = 0;
  virtual int deserialize(unsigned char *data) = 0;
  virtual const char * getType() = 0;
  virtual const char * getMD5() = 0;

  /**
   * @brief This tricky function handles promoting a 32bit float to a 64bit
   *        double, so that AVR can publish messages containing float64
   *        fields, despite AVR having no native support for double.
   *
   * @param[out] outbuffer pointer for buffer to serialize to.
   * @param[in] f value to serialize.
   *
   * @return number of bytes to advance the buffer pointer.
   *
   */
  static int serializeAvrFloat64(unsigned char* outbuffer, const float f)
  {
    int32_t val;
    std::memcpy(&val, &f, sizeof(val));

    int16_t exp = ((val >> 23) & 255);
    uint32_t mantissa = val & 0x7FFFFF;

    if (exp == 255)
    {
      exp = 2047; // Special value for NaN, infinity etc.
    }
    else if (exp != 0)
    {
      exp += 1023 - 127; // Normal case
    }
    else if (!mantissa)
    {
      exp = 0; // Zero
    }
    else
    {
      // Denormalized value in float, will fit as normalized value in double
      exp += 1023 - 127;
      mantissa <<= 1;
      while (!(mantissa & 0x800000))
      {
          mantissa <<= 1;
          exp--;
      }
      mantissa &= 0x7FFFFF;
    }

    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = (mantissa << 5) & 0xff;
    *(outbuffer++) = (mantissa >> 3) & 0xff;
    *(outbuffer++) = (mantissa >> 11) & 0xff;
    *(outbuffer++) = ((exp << 4) & 0xF0) | ((mantissa >> 19) & 0x0F);
    *(outbuffer++) = (exp >> 4) & 0x7F;

    // Mark negative bit as necessary.
    if (f < 0)
    {
      *(outbuffer - 1) |= 0x80;
    }

    return 8;
  }

  /**
   * @brief This tricky function handles demoting a 64bit double to a
   *        32bit float, so that AVR can understand messages containing
   *        float64 fields, despite AVR having no native support for double.
   *
   * @param[in] inbuffer pointer for buffer to deserialize from.
   * @param[out] f pointer to place the deserialized value in.
   *
   * @return number of bytes to advance the buffer pointer.
   */
  static int deserializeAvrFloat64(const unsigned char* inbuffer, float* f)
  {
    int16_t exp;
    uint32_t mantissa;

    // Skip lowest 24 bits
    inbuffer += 3;

    // Copy truncated mantissa.
    mantissa = ((uint32_t)(*(inbuffer++)) >> 4 & 0x0F);
    mantissa |= ((uint32_t)(*(inbuffer++)) & 0xff) << 4;
    mantissa |= ((uint32_t)(*(inbuffer++)) & 0xff) << 12;
    mantissa |= ((uint32_t)(*inbuffer) & 0x0f) << 20;

    // Copy exponent.
    exp = ((uint32_t)(*(inbuffer++)) & 0xf0) >> 4;
    exp |= ((uint32_t)(*inbuffer) & 0x7f) << 4;

    if (exp == 2047)
    {
      exp = 255; // NaN, infinity etc.
    }
    else if (exp - 1023 > 127)
    {
      exp = 255;
      mantissa = 0; // Too large for float, convert to infinity
    }
    else if (exp - 1023 >= -126)
    {
      exp -= 1023 - 127; // Normal case
    }
    else if (exp - 1023 < -150)
    {
      exp = 0; // Too small or zero
    }
    else
    {
      // Have to convert to denormalized representation for float
      mantissa |= 0x1000000;
      mantissa >>= ((-126 + 1023) - exp);
      exp = 0;
    }

    // Round off mantissa
    if (mantissa != 0xFFFFFF)
      mantissa += 1;

    mantissa >>= 1;

    // Put mantissa and exponent into place
    uint32_t val = mantissa;
    val |= static_cast<uint32_t>(exp) << 23;

    // Copy negative sign.
    val |= (static_cast<uint32_t>(*(inbuffer++)) & 0x80) << 24;

    std::memcpy(f, &val, sizeof(val));
    return 8;
  }

  // Copy data from variable into a byte array
  template<typename A, typename V>
  static void varToArr(A arr, const V var)
  {
    for (size_t i = 0; i < sizeof(V); i++)
      arr[i] = (var >> (8 * i));
  }

  // Copy data from a byte array into variable
  template<typename V, typename A>
  static void arrToVar(V& var, const A arr)
  {
    var = 0;
    for (size_t i = 0; i < sizeof(V); i++)
      var |= (arr[i] << (8 * i));
  }

};

}  // namespace ros

#endif
