/*
 * Copyright (c) 2012 Timo Kerstan.  All right reserved.
 *
 * This file is part of Aquaduino.
 *
 * Aquaduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Aquaduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Aquaduino.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <Framework/Aquaduino.h>
#include "SerialAtlasORP.h"
#include <Arduino.h>
#include <SD.h>

#ifdef FEATURE_WEBIF
#include <TemplateParser.h>
#include <Framework/Flashvars.h>
#endif

static bool sensorORP_stringcomplete;
static String sensorstringORP = "";
static double actualORP;

void serialEvent2(){
  char inchar = (char)Serial2.read();
  sensorstringORP += inchar;
  if(inchar == '\r') {
      char tempORP[6];
          sensorstringORP.toCharArray(tempORP,5);
          actualORP = 0;
          actualORP = atof(tempORP);
          sensorstringORP="";
  }
}


/**
 * \brief Constructor
 */
SerialAtlasORP::SerialAtlasORP()
{
    Serial2.begin(38400);
    m_Type = SENSOR_DIGITALINPUT;
    m_Pin = 0;
}

/**
 * \brief Returns the value of the digital input
 *
 * \returns 1 if input is HIGH or 0 if input is LOW.
 */
double SerialAtlasORP::read()
{
    return actualORP;
}

uint16_t SerialAtlasORP::serialize(void* buffer, uint16_t size)
{
    memcpy(buffer, &m_Pin, sizeof(m_Pin));
    return sizeof(m_Pin);
}

uint16_t SerialAtlasORP::deserialize(void* data, uint16_t size)
{
    memcpy(&m_Pin, data, sizeof(m_Pin));
    pinMode(m_Pin, INPUT);
    return sizeof(m_Pin);
}

#ifdef FEATURE_WEBIF
int8_t SerialAtlasORP::showWebinterface(WebServer* server,
                                      WebServer::ConnectionType type, char* url)
{
    File templateFile;
    TemplateParser* parser;
    int16_t matchIdx;
    char templateFileName[template_digitalinput_fnsize];
    strcpy_P(templateFileName, template_digitalinput_fname);

    if (type == WebServer::POST)
    {
        int8_t repeat;
        char name[16], value[16];
        do
        {
            repeat = server->readPOSTparam(name, 16, value, 16);
            if (strcmp_P(name,
                         (PGM_P) pgm_read_word(&(template_digitalinput_inputs[DI_I_PIN])))
                == 0)
            {
                m_Pin = atoi(value);
                pinMode(m_Pin, INPUT);
            }
        } while (repeat);
        server->httpSeeOther(this->m_URL);
    }
    else
    {
        server->httpSuccess();
        parser = __aquaduino->getTemplateParser();
        templateFile = SD.open(templateFileName, FILE_READ);
        while ((matchIdx =
                parser->processTemplateUntilNextMatch(&templateFile,
                                                      template_digitalinput,
                                                      template_digitalinput_elements,
                                                      server))
               >= 0)
        {
            switch (matchIdx)
            {
            case DI_SNAME:
                server->print(getName());
                break;
            case DI_PIN_NAME:
                server->print((__FlashStringHelper *) pgm_read_word(template_digitalinput_inputs));
                break;
            case DI_PIN_VAL:
                server->print(m_Pin);
                break;
            case DI_PIN_SIZE:
                server->print(3);
                break;
            case DI_PIN_MAXLENGTH:
                server->print(2);
                break;

            }
        }

        templateFile.close();
    }
    return true;
}
#endif
