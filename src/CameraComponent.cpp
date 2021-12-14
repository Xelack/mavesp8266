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
 * @file CameraComponent.cpp
 * Novatek Wifi Camera Command
 *
 * @author Xelack
 */

#include "CameraComponent.h"

//---------------------------------------------------------------------------------
//-- Initialize
CameraComponent::CameraComponent(Camera *CameraDevice)
{
  _Camera = CameraDevice;
}

const char *CameraComponent::getName()
{
  return CAMERA_COMPONENT_NAME;
}

bool CameraComponent::handleMessage(MavESP8266Bridge *sender, mavlink_message_t *message)
{
  if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT)
  {
    return false;
  }
  if (message->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
  {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(message, &cmd);
    switch (cmd.command)
    {
    case MAV_CMD_DO_DIGICAM_CONTROL:
      DEBUG_LOG("MAV_CMD_DO_DIGICAM_CONTROL {\n");
      break;
    case MAV_CMD_REQUEST_CAMERA_SETTINGS:
      DEBUG_LOG("MAV_CMD_REQUEST_CAMERA_SETTINGS {\n");
      break;
    case MAV_CMD_SET_CAMERA_MODE:
      DEBUG_LOG("MAV_CMD_SET_CAMERA_MODE {\n");
      if (_Camera->setMode((CAMERA_MODE)cmd.param2))
      {
        send_feedback();
      }
      break;
    case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
      DEBUG_LOG("MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS {\n");
      break;
    case MAV_CMD_IMAGE_START_CAPTURE:
      break;
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
      DEBUG_LOG("MAV_CMD_DO_DIGICAM_CONFIGURE {\n");
      break;
    case MAV_CMD_DO_TRIGGER_CONTROL:
      DEBUG_LOG("MAV_CMD_DO_TRIGGER_CONTROL {\n");
      break;
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
      DEBUG_LOG("MAV_CMD_DO_SET_CAM_TRIGG_DIST {\n");
      break;
    default:
      break;
    }
  }
  else
  {
    switch (message->msgid)
    {
    case MAVLINK_MSG_ID_CAMERA_INFORMATION:
      DEBUG_LOG("MAVLINK_MSG_ID_CAMERA_INFORMATION\n");
      break;
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
      DEBUG_LOG("MAVLINK_MSG_ID_DIGICAM_CONFIGURE\n");
      break;

    case MAVLINK_MSG_ID_CAMERA_SETTINGS:
      DEBUG_LOG("MAVLINK_MSG_ID_CAMERA_SETTINGS\n");
      break;
    case MAVLINK_MSG_ID_CAMERA_TRIGGER:
      DEBUG_LOG("MAVLINK_MSG_ID_CAMERA_TRIGGER\n");
      break;
    case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
      DEBUG_LOG("MAVLINK_MSG_ID_CAMERA_FEEDBACK\n");
      break;

    case MAVLINK_MSG_ID_CAMERA_STATUS:
      DEBUG_LOG("MAVLINK_MSG_ID_CAMERA_STATUS\n");
      break;
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
      DEBUG_LOG("MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
      break;
    case MAVLINK_MSG_ID_RC_CHANNELS:
      DEBUG_LOG("MAVLINK_MSG_ID_RC_CHANNELS\n");
      //     mavlink_rc_channels_t rc_channels;
      //     mavlink_msg_rc_channels_decode(message, &rc_channels);
      //     DEBUG_LOG("RC %u\n", rc_channels.chan8_raw);
      break;

    default:
      break;
    }
  }
  return false;
}

bool CameraComponent::send_feedback()
{
  return true;
}
