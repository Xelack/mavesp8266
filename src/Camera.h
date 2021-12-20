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
 * @file Camera.h
 * Camera base interface class
 * @author Alexandre PRIETO (Xelack)
 */

#ifndef CAMERA_H
#define CAMERA_H

class Camera;
#ifndef HAVE_ENUM_CAPTURE_MODE
#define HAVE_ENUM_CAPTURE_MODE
typedef enum CAPTURE_MODE
{
   IMAGE=0, /* Camera is in image/photo capture mode. | */
   VIDEO=1, /* Camera is in video capture mode. | */
   IMAGE_SURVEY=2, /* Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys. | */
   MODE_ENUM_END=3, /*  | */
} CAPTURE_MODE;
#endif

//---------------------------------------------------------------------------------
//-- Base Camera
class Camera {
public:
    virtual bool    begin           () = 0;
    virtual int     takePicture     (char * filename) = 0;
    virtual int     takePicture     () = 0;
    virtual int     startRec        () = 0;
    virtual int     stopRec         () = 0;
    virtual int     setMode         (CAPTURE_MODE Mode) = 0;
    virtual CAPTURE_MODE    getMode  () = 0;
    virtual bool    isReady         () = 0;
    virtual bool    isRecording     () = 0;
};

#endif