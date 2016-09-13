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

#include "Actuator.h"
#include <Framework/Aquaduino.h>
#include <Time.h>

/**
 * \brief Constructor
 * \param name Name of the actuator.
 *
 * The name is copied into the object. Maximum size is AQUADUINO_STRING_LENGTH.
 */
Actuator::Actuator(const char* name) :
		m_locked(1) {
	m_ControlledBy = -1;
	setName(name);
}

/**
 * \brief Destructor.
 *
 * Empty.
 */
Actuator::~Actuator() {
}

/**
 * \brief Sets the controller this actuator is assigned to.
 * \param[in] controller Index of the controller this actuator is assigned
 *                       to.
 *
 * Sets the index of the controller this actuator is assigned to. The index of
 * the controller is available through getControllerID of the Aquaduino class.
 */
void Actuator::setController(int8_t controller) {
	this->m_ControlledBy = controller;
}

/**
 * \brief Gets the index of the controller this actuator is assigned to.
 *
 * \returns Index of the controller this actuator is assigned to.
 */
int8_t Actuator::getController() {
	return m_ControlledBy;
}

/**
 * \brief Locks the usage of this actuator.
 *
 * The actuator is locked and its state shall not be modifiable by the methods
 * on, off and setPWM.
 */
void Actuator::lock() {
	m_locked = true;
}

/**
 * \brief Unlocks the usage of this actuator.
 *
 * The actuator is unlocked and its state can be modified by the methods
 * on, off and setPWM.
 */
void Actuator::unlock() {
	m_locked = false;
}

/**
 * \brief Checks whether the usage of this actuator is locked or not.
 *
 * \returns True when locked. Otherwise false.
 */
int8_t Actuator::isLocked() {
	return m_locked;
}

/**
 * \brief sets the operating hours (seconds)
 *
 */
void Actuator::setOperatingTime(uint32_t seconds) {
	m_opTime = seconds;
}
/**
 * \brief sets the last reset operating hours (seconds)
 *
 */
void Actuator::setLastResetOperatingTime(uint32_t seconds) {
	m_opResetTime = seconds;
}
/**
 * \brief resets the operating hours (seconds since 1970)
 *
 */
void Actuator::resetOperatingTime() {
	m_opResetTime = now();
	m_opTime = 0;
	__aquaduino->writeOperatingHours(__aquaduino);

}
/**
 * \brief returns the last operating hours reset
 *
 */
uint32_t Actuator::getLastResetOperatingTime() {
	return m_opResetTime;

}

uint32_t Actuator::getOperatingTime() {
	Serial.println("! override problem in Actuator::getOperatingTime");
	return 0;

}
