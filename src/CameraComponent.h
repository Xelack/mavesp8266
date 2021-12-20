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
 * @file CameraComponent.h
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Alexandre PRIETO (Xelack)
 */

#ifndef CAMERACOMPONENT_H
#define CAMERACOMPONENT_H

#include <ardupilotmega/mavlink.h>

#include "PComponent.h"
#include "Camera.h"

class CameraComponent : public PComponent {
    public:
        CameraComponent(Camera * CameraDevice);

        //- Returns true if the component consumed the message
        bool handleMessage          (MavESP8266Bridge* sender, mavlink_message_t* message);
        bool begin                  ();
        bool send_feedback          ();
        const char * getName();
        
    private:
        Camera * _Camera;
};


#endif
