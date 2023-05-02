/*
 * Software License Agreement (BSD License)
 *
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

#ifndef MKR1000HARDWARE_H
#define MKR1000HARDWARE_H

#include <WiFi101.h>

class MKR1000Hardware {
  public:
    MKR1000Hardware()
    {
    }

    void setConnection(IPAddress &server, int port = 11411) {
      this->server = server;
      this->serverPort = port;
    }

    IPAddress getLocalIP() {
      return WiFi.localIP();
    }

    void init() {
      this->tcp.connect(this->server, this->serverPort);
    }

    int read() {
      if (this->tcp.connected()) {
        return tcp.read();
      } else {
        this->tcp.connect(this->server, this->serverPort);
      }
      return -1;
    };

    void write(const uint8_t* data, size_t length) {
      tcp.write(data, length);
    }

    unsigned long time() {return millis();}

  protected:
    WiFiClient tcp;
    IPAddress server;
    uint16_t serverPort = 11411;
};

#endif  // MKR1000HARDWARE_H
