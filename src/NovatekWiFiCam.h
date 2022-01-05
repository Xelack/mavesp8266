/****************************************************************************
 *
 * Copyright (c) 2021, 2022 Alexandre PRIETO (Xelack). All rights reserved.
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
 * @file NovatekWiFiCam.h
 * Novatek Wifi Camera Command
 * @author Alexandre PRIETO (Xelack)
 */

#ifndef NOVATEKWIFICAM_H
#define NOVATEKWIFICAM_H

//#define ENABLE_DEBUG

#include "Arduino.h"

#include "Camera.h"
#ifndef ESP32
    #include <WiFiClient.h>
    #include <esp8266httpclient.h>
#else
     #include <HTTPClient.h>
#endif

#include "tinyxml2.h"
 
using namespace tinyxml2;
#include <string>

/*Begin of HTTP Protocole definitions *************************************/

#define DEFAULT_BASE_URL       "http://192.168.1.254"
#define DEFAULT_PHOTO_PATH     "/DCIM/PHOTO/"
#define DEFAULT_VIDEO_PATH     "/DCIM/MOVIE/"
#define WIFIAPP_CMD_SUFFIX     "/?custom=1"
#define CMD                    "&cmd="
#define PAR                    "&par="
#define WIFIAPP_CMD_CAPTURE    "1001"
#define WIFIAPP_CMD_MODECHANGE "3001"
#define WIFIAPP_CMD_RECORD     "2001"

#define START                  "1"
#define STOP                   "0"
/*End of HTTP Protocole definitions *************************************/


class NovatekWiFiCam : public Camera {
public:
    NovatekWiFiCam          ();
    bool    begin           ();
    int     takePicture     (char * filename);
    int     takePicture     ();
    int     startRec        ();
    int     stopRec         ();
    bool    isRecording     ();
    int     setMode         (CAPTURE_MODE Mode);
    CAPTURE_MODE getMode     ();
    bool  isReady           ();
    

private:
    void     _buildUrl       (const char* Command, const char * Parameter);
    int      _httpGet        ();
    XMLElement * _parseResponse(const char * searchForParentElement,
                                const char * searchForChildElement = "", 
                                const char * searchForSubElement = "");
    bool                _ready;
    bool                _idle;
    bool                _recording;
    String              _url;
    String              _response;
    HTTPClient          _http;
#ifndef ESP32
    WiFiClient          _WiFiClient;
#endif
    CAPTURE_MODE       _Mode;
    XMLDocument       xmlDocument;     
};

#endif
