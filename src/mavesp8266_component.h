/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_component.h
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */
// Macro to define packed structures
#ifdef __GNUC__
  #define STRUCPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define STRUCPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#ifndef MAVESP8266_COMPONENT_H
#define MAVESP8266_COMPONENT_H

#include "mavesp8266.h"
#include "PComponent.h"
#ifdef ESP32
  #include <freertos/queue.h>
  #include <freertos/task.h>
#endif

#define MAX_PERIPHERALS 2

#ifdef ESP32
  #define PERIPHERALS_QUEUE_LEN 10

  STRUCPACKED(
  typedef struct __peripheralsqueue {
      MavESP8266Bridge *sender;
      MavESP8266Component *comp;
      mavlink_message_t Message;
  }) PeripheralsQueue_t;
#endif

class MavESP8266Component {
public:
    MavESP8266Component();

    //- Returns true if the component consumed the message
    bool handleMessage          (MavESP8266Bridge* sender, mavlink_message_t* message);
    bool inRawMode              ();
    void resetRawMode           () { _in_raw_mode_time = millis(); }
    int  sendMsgToGCS           (const char* text);
    int  addPeripheral          (PComponent * device);
    PComponent * getPeripheral  (int index);
    PComponent * getPeripheral  (const char * name);
    PComponent **       getPeripherals  ();
    int                 getPeripheralsCount();
    void rebootDevice           ();

private:
    int                 _sendStatusMessage      (MavESP8266Bridge* sender, uint8_t type, const char* text);
    void                _handleParamSet         (MavESP8266Bridge* sender, mavlink_param_set_t* param);
    void                _handleParamRequestList (MavESP8266Bridge* sender);
    void                _handleParamRequestRead (MavESP8266Bridge* sender, mavlink_param_request_read_t* param);
    void                _sendParameter          (MavESP8266Bridge* sender, uint16_t index);
    void                _sendParameter          (MavESP8266Bridge* sender, const char* id, uint32_t value, uint16_t index);

    void                _handleCmdLong          (MavESP8266Bridge* sender, mavlink_command_long_t* cmd, uint8_t compID);

    void                _wifiReboot             (MavESP8266Bridge* sender);

    bool                _in_raw_mode;
    unsigned long       _in_raw_mode_time;

    //TaskFunction_t      _PeripheralsTask(void* parameters);
    int                 _periph_count = 0;
    PComponent *        _Peripherals[MAX_PERIPHERALS];
    #ifdef ESP32
    PeripheralsQueue_t  _PeripheralsQueueItem;
    #endif
};

#endif
