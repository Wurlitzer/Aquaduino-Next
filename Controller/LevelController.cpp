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

#include <Controller/LevelController.h>
#include <Aquaduino.h>
#include <SD.h>
#include <limits.h>

enum TEMPLATE_LEVELCONTROLLER_STATES
{
    LEVELCONTROLLER_STATE_OK,
    LEVELCONTROLLER_STATE_DEBOUNCE,
    LEVELCONTROLLER_STATE_REFILL,
    LEVELCONTROLLER_STATE_OVERRUN,
    LEVELCONTROLLER_STATE_REFILL_TIMEOUT
};

/**
 * \brief Constructor
 * \param[in] name Name of the controller
 */
LevelController::LevelController(const char* name) :
        Controller(name)
{
    m_Type = CONTROLLER_LEVEL;
    m_Delayh = 3;
    m_Delayl = 10;
    m_State = LEVELCONTROLLER_STATE_OK;
    m_Timeout = 30;
    m_Sensor = -1;
}

uint16_t LevelController::serialize(void* buffer, uint16_t size)
{
    uint8_t* bPtr = (uint8_t*) buffer;
    uint8_t offset = 0;

    uint16_t mySize = sizeof(m_Delayl) + sizeof(m_Delayh)
                      + sizeof(m_Timeout) + sizeof(m_Sensor);
    if (mySize <= size)
    {
        memcpy(bPtr, &m_Sensor, sizeof(m_Sensor));
        offset += sizeof(m_Sensor);
        memcpy(bPtr + offset, &m_Delayl, sizeof(m_Delayl));
        offset += sizeof(m_Delayl);
        memcpy(bPtr + offset, &m_Delayh, sizeof(m_Delayh));
        offset += sizeof(m_Delayh);
        memcpy(bPtr + offset, &m_Timeout, sizeof(m_Timeout));
        offset += sizeof(m_Timeout);
        return mySize;
    }
    return 0;
}

uint16_t LevelController::deserialize(void* data, uint16_t size)
{
    uint8_t* bPtr = (uint8_t*) data;
    uint8_t offset = 0;

    uint16_t mySize = sizeof(m_Delayl) + sizeof(m_Delayh)
                      + sizeof(m_Timeout);
    if (mySize <= size)
    {
        memcpy(&m_Sensor, bPtr, sizeof(m_Sensor));
        offset += sizeof(m_Sensor);
        if (m_Sensor < 0 || m_Sensor >= MAX_SENSORS)
            m_Sensor = -1;
        memcpy(&m_Delayl, bPtr + offset, sizeof(m_Delayl));
        offset += sizeof(m_Delayl);
        memcpy(&m_Delayh, bPtr + offset, sizeof(m_Delayh));
        offset += sizeof(m_Delayh);
        memcpy(&m_Timeout, bPtr + offset, sizeof(m_Timeout));
        offset += sizeof(m_Timeout);
        return mySize;
    }
    return 0;
}

/**
 * \brief Run method triggered by Aquaduino::run
 *
 * This method implements a state machine for the refill process.
 *
 * When in state LEVELCONTROLLER_STATE_OK the state machine waits for
 * the reading to become HIGH and performs a transition to
 * LEVELCONTROLLER_STATE_DEBOUNCE.
 *
 * In state LEVELCONTROLLER_STATE_DEBOUNCE it is checked if the reading becomes
 * LOW within the time LevelController::debounceDelayHigh seconds. When this is
 * the case a transition back to LEVELCONTROLLER_STATE_OK is performed. This is
 * the case when the level sensor detected a wave. When the signal is HIGH for
 * at least LevelController::debounceDelayHigh seconds a transition to
 * LEVELCONTROLLER_STATE_REFILL is performed and all assigned actuators are
 * activated.
 *
 * In state LEVELCONTROLLER_STATE_REFILL it is checked whether the signal
 * remains HIGH for more than LevelController::refillTimeout seconds. If thats
 * the case it is assumed that the water reservoir ran out of water and the
 * refill process is stopped and a transition to
 * LevelController::LEVELCONTROLLER_STATE_REFILL_TIMEOUT is performed.
 * If the signal gets LOW within LevelController::refillTimeout seconds a
 * transition to LEVELCONTROLLER_STATE_OVERRUN is performed.
 *
 * In state LEVELCONTROLLER_STATE_OVERRUN it is checked whether the reading
 * remains LOW and a time of LevelController::hysteresis seconds passes. If
 * this is the case the refill process has finished and a transition to
 * LEVELCONTROLLER_STATE_OK is performed. If the reading gets HIGH again
 * a transition back to LEVELCONTROLLER_STATE_REFILL.
 *
 * In state LEVELCONTROLLER_STATE_REFILL_TIMEOUT nothing happens until
 * the state machine variables are reseted from outside this function. This
 * reset is currently implemented in the POST request processing in
 * LevelController::showWebinterface.
 *
 */
int8_t LevelController::run()
{
    static unsigned long lastTime = 0;
    unsigned long millisNow = 0;
    long delay_low_millis = 0;
    long delay_high_millis = 0;
    long timeout_millis = 0;
    long deltaTSwitch = 0;
    int8_t sensor_val = 0;

    if (m_Sensor < 0 || m_Sensor >= MAX_SENSORS)
        return -1;

    sensor_val = __aquaduino->getSensorValue(m_Sensor);
    millisNow = millis();
    deltaTSwitch = millisNow - lastTime;

    delay_high_millis = 1000 * (long) m_Delayh;
    delay_low_millis = 1000 * (long) m_Delayl;
    timeout_millis = 1000 * (long) m_Timeout;

    //Check for overflow while processing refill
    if (m_State != LEVELCONTROLLER_STATE_OK && lastTime > millisNow)
        deltaTSwitch = LONG_MAX - lastTime + millisNow;

    switch (m_State)
    {
    case LEVELCONTROLLER_STATE_OK:
        allMyActuators((int8_t) 0);
        if (sensor_val == HIGH)
        {
            m_State = LEVELCONTROLLER_STATE_DEBOUNCE;
            lastTime = millisNow;
        }
        break;
    case LEVELCONTROLLER_STATE_DEBOUNCE:
        if (sensor_val == HIGH && deltaTSwitch > delay_high_millis)
        {
            allMyActuators((int8_t) 1);
            lastTime = millisNow;
            m_State = LEVELCONTROLLER_STATE_REFILL;
        }
        else if (sensor_val == LOW)
        {
            m_State = LEVELCONTROLLER_STATE_OK;
        }
        break;
    case LEVELCONTROLLER_STATE_REFILL:
        if (sensor_val == LOW)
        {
            m_State = LEVELCONTROLLER_STATE_OVERRUN;
            lastTime = millisNow;
        }
        else if (sensor_val == HIGH && deltaTSwitch > timeout_millis)
        {
            m_State = LEVELCONTROLLER_STATE_REFILL_TIMEOUT;
            allMyActuators((int8_t) 0);
        }
        break;
    case LEVELCONTROLLER_STATE_OVERRUN:
        if (sensor_val == LOW && deltaTSwitch > delay_low_millis)
        {
            m_State = LEVELCONTROLLER_STATE_OK;
            allMyActuators((int8_t) 0);
        }
        else if (sensor_val == HIGH)
        {
            m_State = LEVELCONTROLLER_STATE_REFILL;
        }
        break;
    case LEVELCONTROLLER_STATE_REFILL_TIMEOUT:
        return 0;
    default:
        return 0;
    }
    return 1;
}

