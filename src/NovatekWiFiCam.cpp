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
 * @file NovatekWiFiCam.cpp
 * Novatek Wifi Camera Command
 *
 * @author Xelack
 */

#include "NovatekWiFiCam.h"
#include "mavesp8266.h"

//---------------------------------------------------------------------------------
//-- Initialize
NovatekWiFiCam::NovatekWiFiCam()
{
}

bool NovatekWiFiCam::begin()
{
  _idle = true;
  _recording = false;
  _Mode = CAPTURE_MODE::MODE_ENUM_END;
  int httpResponseCode = setMode(CAPTURE_MODE::IMAGE);
  _ready = (httpResponseCode > 0);
  return _ready;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool NovatekWiFiCam::isReady()
{
  return _ready & _idle;
}

/**
 * @brief
 *
 * @return const char*
 */
CAPTURE_MODE NovatekWiFiCam::getMode()
{
  return _Mode;
}

/**
 * @brief
 *
 * @param Mode
 * @return int
 */
int NovatekWiFiCam::setMode(CAPTURE_MODE Mode)
{
  if (getMode() == Mode)
  {            // Check Current Mode with Mode requested
    return -1; // do nothing if is already set
  }
  if (_recording){ //recording in progress, stop 
    if (stopRec() < 0){ return -2;}
  }
  _buildUrl(WIFIAPP_CMD_MODECHANGE, ((Mode == CAPTURE_MODE::IMAGE)? "0": "1"));
  // Send HTTP GET request
  int httpResponseCode = _httpGet();
  if (httpResponseCode >= 0)
  {
    _Mode = Mode;
  }
  return httpResponseCode;
}

/**
 * @brief
 *
 * @return int
 */
int NovatekWiFiCam::takePicture(char * filename)
{
  _buildUrl(WIFIAPP_CMD_CAPTURE, "");
  // Send HTTP GET request
  int statusRequest = _httpGet();
  if (statusRequest >= 0)
  {
    XMLElement * pElement = _parseResponse("Function", "File", "NAME");
    if (pElement != nullptr)
    {
      // strncpy(filename, pElement->GetText(), sizeof(filename));
      memcpy(filename,  pElement->GetText(), (size_t) (sizeof(char) * strlen(filename)));
    }
  }
  return statusRequest;
}
int NovatekWiFiCam::takePicture()
{
  DEBUG_LOG("takePicture\n");
  _buildUrl(WIFIAPP_CMD_CAPTURE, "");
  // Send HTTP GET request
  return _httpGet();
}
/**
 * @brief
 *
 * @return int
 */
int NovatekWiFiCam::startRec()
{
  _buildUrl(WIFIAPP_CMD_RECORD, START);
  // Send HTTP GET request
  int httpResponseCode = _httpGet();
  if(httpResponseCode >= 0){
    _recording = true;
  }
  return httpResponseCode;
}

/**
 * @brief
 *
 * @return int
 */
int NovatekWiFiCam::stopRec()
{
  _buildUrl(WIFIAPP_CMD_RECORD, STOP);
  // Send HTTP GET request
  int httpResponseCode = _httpGet();
  if(httpResponseCode >= 0){
    _recording = false;
  }
  return httpResponseCode;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool NovatekWiFiCam::isRecording(){
  return _recording;
}
/**
 * @brief
 *
 * @param Command
 * @param Parameter
 */
void NovatekWiFiCam::_buildUrl(const char *Command, const char *Parameter)
{
  _url = DEFAULT_BASE_URL;
  _url += WIFIAPP_CMD_SUFFIX;
  _url += CMD;
  _url += Command;
  if (strlen(Parameter) != 0)
  {
    _url += PAR;
    _url += Parameter;
  }
}

/**
 * @brief
 *
 * @return int
 */
int NovatekWiFiCam::_httpGet()
{
  if (!_idle)
  {
    // Wait a few seconds
    for (int i = 0; i < 120 && !_idle; i++)
    {
      delay(100);
    }
    if (!_idle)
      return -1;
  }
  _idle = false;
  int httpResponseCode = -1;
  _http.begin(_url.c_str());
  httpResponseCode = _http.GET();
  _response = _http.getString();
  if (httpResponseCode > 0)
  {
    xmlDocument.Clear();
    if (xmlDocument.Parse(_response.c_str()) == XML_SUCCESS)
    {
      XMLElement *pElement = _parseResponse("Function", "Status");
      if (pElement != nullptr)
      {
        pElement->QueryIntText(&httpResponseCode);
      }
    }
  }
  // Free resources
  _http.end();
  _idle = true;
  return httpResponseCode;
}

XMLElement *NovatekWiFiCam::_parseResponse(const char *searchForParentElement,
                                           const char *searchForChildElement,
                                           const char *searchForSubElement)
{
  XMLNode *pRoot = xmlDocument.LastChild();
  if (pRoot != nullptr)
  {
    if (strcmp(searchForParentElement, pRoot->Value()) == 0)
    {
      if (strlen(searchForChildElement) > 0)
      {
        XMLElement *pElement = pRoot->FirstChildElement(searchForChildElement);
        if (pElement != nullptr)
        {
          if (strlen(searchForSubElement) > 0)
          {
            pElement = pElement->FirstChildElement(searchForSubElement);
          }
          if (pElement != nullptr)
          {
            return pElement;
          }
        }
      }
      else
      {
        return pRoot->ToElement();
      }
    }
  }
  return nullptr;
}
