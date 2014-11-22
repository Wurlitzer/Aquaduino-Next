/*
 * GUIServer.cpp
 *
 *  Created on: 12.03.2014
 *      Author: Timo
 */

#include <Framework/Aquaduino.h>
#include <Framework/GUIServer.h>
#include <Arduino.h>
#include <Controller/ClockTimerController.h>
#include <Controller/TemperatureController.h>
#include <Controller/LevelController.h>
#include <Actuators/DigitalOutput.h>
#include <Sensors/DS18S20.h>
#include <Sensors/DigitalInput.h>
#include <OneWireHandler.h>

enum {
	GET_VERSION = 0,
	GET_ALL_SENSORS = 1,
	GET_SENSOR_DATA = 2,
	SET_SENSOR_CONFIG = 3,
	GET_ALL_ACTUATORS = 4,
	GET_ACTUATOR_DATA = 5,
	SET_ACTUATOR_DATA = 6,
	SET_ACTUATOR_CONFIG = 7,
	GET_ALL_CONTROLLERS = 8,
	GET_CLOCK_TIMERS = 9,
	SET_CLOCK_TIMER = 10,
	GET_TEMPSENSORS_AT_PIN = 11,
	SET_TEMPSENSOR_AT_PIN = 12,
	GET_TEMPERATURE_CONTROLLER = 13,
	SET_TEMPERATURE_CONTROLLER = 14,
	GET_LEVEL_CONTROLLER = 15,
	SET_LEVEL_CONTROLLER = 16,
	RESET_LEVEL_CONTROLLER = 17,
	SET_LEVEL_CONTROLLER_NAME = 18,
	SET_TEMPERATURE_CONTROLLER_NAME = 19,
	GET_DS1820_ADDRESSES = 20,
	SET_DS1820_ADDRESS = 21
};

GUIServer::GUIServer(uint16_t port) {
	m_Port = port;
	m_UdpServer.begin(m_Port);
	Serial.println("GUIServer Start");
}

GUIServer::~GUIServer() {
}

int8_t GUIServer::receiveCommand() {
	if (!m_UdpServer.parsePacket()) {
		return 0;
	}
	Serial.println(("."));
	Serial.print(F("UDP Packet of "));
	Serial.print(m_UdpServer.available());
	Serial.print(F(" Bytes available. "));
	Serial.print(F("Got "));
	Serial.print(m_UdpServer.read(m_Buffer, sizeof(m_Buffer)));
	Serial.println(F(" Bytes"));
	return 1;
}
extern int freeRam();
void GUIServer::run() {
	if (receiveCommand()) {
		m_UdpServer.beginPacket(m_UdpServer.remoteIP(),
				m_UdpServer.remotePort());
		//send back methodID
		m_UdpServer.write(m_Buffer[1]);
		//send back requestID
		m_UdpServer.write(m_Buffer[0]);

		//trace
		Serial.print(F("Request ID: "));
		Serial.print(m_Buffer[0]);
		Serial.print(F("  Method ID: "));
		Serial.print(m_Buffer[1]);
		Serial.print(F(" value: "));
		Serial.print(m_Buffer[2]);
		Serial.print(F(" to: "));
		Serial.println(m_UdpServer.remoteIP());

		//Serial.println(freeRam());

		//switch to method
		switch (m_Buffer[1]) {
		case GET_VERSION:
			m_UdpServer.write((uint8_t) 0);
			m_UdpServer.write(2);
			break;
		case GET_ALL_SENSORS:
			getAllSensors();
			break;
		case GET_SENSOR_DATA:
			getSensorData(m_Buffer[2]);
			break;
		case SET_SENSOR_CONFIG:
			setSensorConfig(m_Buffer[2]);
			break;
		case GET_ALL_ACTUATORS:
			getAllActuators();
			break;
		case GET_ACTUATOR_DATA:
			getActuatorData(m_Buffer[2]);
			break;
		case SET_ACTUATOR_DATA:
			setActuatorData(m_Buffer[2]);
			break;
		case SET_ACTUATOR_CONFIG:
			setActuatorConfig(m_Buffer[2]);
			break;
		case GET_ALL_CONTROLLERS:
			getAllControllers();
			break;
		case GET_CLOCK_TIMERS:
			getClockTimers(m_Buffer[2]);
			break;
		case SET_CLOCK_TIMER:
			setClockTimer(m_Buffer[2]);
			break;
		case GET_TEMPERATURE_CONTROLLER:
			getTemperatureController(m_Buffer[2]);
			break;
		case SET_TEMPERATURE_CONTROLLER:
			setTemperatureController(m_Buffer[2]);
			break;
		case SET_TEMPERATURE_CONTROLLER_NAME:
			setTemperatureControllerName(m_Buffer[2]);
			break;
		case GET_LEVEL_CONTROLLER:
			getLevelController(m_Buffer[2]);
			break;
		case SET_LEVEL_CONTROLLER:
			setLevelController(m_Buffer[2]);
			break;
		case RESET_LEVEL_CONTROLLER:
			resetLevelController(m_Buffer[2]);
			break;
		case SET_LEVEL_CONTROLLER_NAME:
			setLevelControllerName(m_Buffer[2]);
			break;
		case GET_DS1820_ADDRESSES:
			getDS1820Addresses();
			break;
		case SET_DS1820_ADDRESS:
			setDS1820Address(m_Buffer[2]);
			break;

		default:
			break;
		}
		m_UdpServer.endPacket();
	}

}
void changeActuatorAssignment(int8_t oldActuatorID, int8_t newActuatorID,
		int8_t controllerID) {
	Serial.print("changeActuatorAssignement: ");
	Serial.print(oldActuatorID);
	Serial.println(newActuatorID);
	Controller* controller = __aquaduino->getController(controllerID);

	if (oldActuatorID != newActuatorID) {
		Actuator* actuator;
		if (oldActuatorID != -1) {
			actuator = __aquaduino->getActuator(oldActuatorID);
			actuator->setController(-1);
			Serial.println("unset");
			__aquaduino->writeConfig(actuator);
			Serial.println("done");
		}
		if (newActuatorID != -1) {
			actuator = __aquaduino->getActuator(newActuatorID);
			actuator->setController(controllerID);
			Serial.println("set");
			__aquaduino->writeConfig(actuator);
			Serial.println("done");
		}
	}

}
////////////////////////////////
//Sensor
////////////////////////////////
void GUIServer::getAllSensors() {
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//num of sensors
	m_UdpServer.write((uint8_t) __aquaduino->getNrOfSensors());

	//sensor information
	Sensor* sensor;
	__aquaduino->resetSensorIterator();
	while (__aquaduino->getNextSensor(&sensor) != -1) {
		m_UdpServer.write(__aquaduino->getSensorID(sensor));
		//Name:String
		m_UdpServer.write(strlen(sensor->getName()));
		m_UdpServer.write(sensor->getName());
		//Type:int
		m_UdpServer.write(sensor->getType());
		//Unit:String
		m_UdpServer.write((uint8_t) 8);
		m_UdpServer.write("TestUnit");
		//visible:Boolean
		m_UdpServer.write(true);
		//calibrationInterval(days):int
		m_UdpServer.write((uint8_t) 0);
	}
}

void GUIServer::getSensorData(uint8_t sensorId) {

	Serial.print("getSensorData for SensorID: ");
	Serial.println(sensorId);

	Sensor* sensor = __aquaduino->getSensor(sensorId);

	if (sensor) {
		//errorcode 0
		m_UdpServer.write((uint8_t) 0);

		//sensorId:int
		m_UdpServer.write(sensorId);

		//valueAct:float * 1000 -> uint32
		//write((uint32_t) (__aquaduino->getSensorValue(sensorId) * 1000),&m_UdpServer);
		uint32_t tmp = __aquaduino->getSensorValue(sensorId) * 1000;
		m_UdpServer.write((uint8_t*) &tmp, sizeof(int32_t));

		//lastCalibration:dateTime
		tmp = 1415112618;
		m_UdpServer.write((uint8_t*) &tmp, sizeof(int32_t));

		//operatingHours:int
		tmp = 500;
		m_UdpServer.write((uint8_t*) &tmp, sizeof(int32_t));

		//lastOperatingHoursReset
		tmp = 1415112618;
		m_UdpServer.write((uint8_t*) &tmp, sizeof(int32_t));

	} else {
		//errorcode 10 -> sensor not available
		m_UdpServer.write((uint8_t) 10);

	}

}
void GUIServer::setSensorConfig(uint8_t sensorId) {

	Sensor* sensor = __aquaduino->getSensor(sensorId);

	uint8_t type = m_Buffer[3];
	if (sensor) {
		if (type == 1) {
			//sensor->resetOperatinHours();
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 2) {
			sensor->setName((char*) &m_Buffer[4]);
			__aquaduino->writeConfig(sensor);
			//errorcode 0
			m_UdpServer.write((uint8_t) 0);
		}
		if (type == 3) {
			//sensorUnit
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 4) {
			//sensor->setVisible(visible)
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 5) {
			// sensor->setCalibratioInterval(value)
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);

	}

}

////////////////////////////////
//Actuator
////////////////////////////////
void GUIServer::getAllActuators() {
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//num of actuators
	m_UdpServer.write((uint8_t) __aquaduino->getNrOfActuators());

	//actuator information
	Actuator* actuator;
	__aquaduino->resetActuatorIterator();
	while (__aquaduino->getNextActuator(&actuator) != -1) {
		m_UdpServer.write(__aquaduino->getActuatorID(actuator));
		m_UdpServer.write(strlen(actuator->getName()));
		m_UdpServer.write(actuator->getName());
		//influencesStream:bool
		m_UdpServer.write((uint8_t) 0);
		//influencesHeat:bool
		m_UdpServer.write((uint8_t) 0);
		//ControllerSemanticValue:int
		m_UdpServer.write((uint8_t) 0);
		//calibrationInterval(days):int
		m_UdpServer.write((uint8_t) 0);
	}

}

void GUIServer::getActuatorData(uint8_t actuatorId) {
	Serial.print("getActuatorData for ActuatorId: ");
	Serial.println(actuatorId);
	Actuator* actuator = __aquaduino->getActuator(actuatorId);

	if (actuator) {
		//errorcode 0
		m_UdpServer.write((uint8_t) 0);
		//actuatorID:int
		m_UdpServer.write(actuatorId);
		//isOn:0/1
		m_UdpServer.write(actuator->isOn());
		//PWM:0-100
		m_UdpServer.write(actuator->getPWM());
		//isLocked:int
		m_UdpServer.write(actuator->isLocked());
		//operatingHours:int
		m_UdpServer.write((uint8_t) 0);
		//lastOperatingHoursReset:dateTime
		m_UdpServer.write((uint32_t) 1395867979);
		//lastCalibration:dateTime
		m_UdpServer.write((uint32_t) 1395867979);
		//getControllerID
		m_UdpServer.write(actuator->getController());
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);

	}

}

void GUIServer::setActuatorConfig(uint8_t actuatorId) {
	Actuator* actuator = __aquaduino->getActuator(actuatorId);
	uint8_t type = m_Buffer[3];
	if (actuator) {
		if (type == 1) {
			//actuator->resetOperatingHours();
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 2) {
			actuator->setName((char*) &m_Buffer[4]);

			__aquaduino->writeConfig(actuator);
			//errorcode 0
			m_UdpServer.write((uint8_t) 0);
		}
		if (type == 3) {
			//actuator->influenceBitmask(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 4) {
			//actuator->controllerSemanticValue(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 5) {
			//actuator->calibrationInterval(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (type == 6) {
			//actuator->assignedControllerID(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		//__aquaduino->writeConfig(actuator);
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);
	}
}
void GUIServer::setActuatorData(uint8_t actuatorId) {
	//Actuator* actuator = __aquaduino->getActuator(actuatorId);
	DigitalOutput* actuator = (DigitalOutput*) __aquaduino->getActuator(
			actuatorId);

	if (actuator) {
		uint8_t locked = m_Buffer[3];
		uint8_t on = (uint8_t) m_Buffer[4];
		uint8_t pwm = (uint8_t) m_Buffer[5];

		actuator->unlock();
		if (on) {
			actuator->on();
		} else {
			actuator->off();
		}
		if (locked) {
			actuator->lock();
		} else {
			actuator->unlock();
		}
		if (pwm) {
			actuator->setPWM(pwm);
		}
		__aquaduino->writeConfig(actuator);

		//errorcode 0
		m_UdpServer.write((uint8_t) 0);
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);

	}

}
///////////////////////////
//Controller
///////////////////////////
void GUIServer::getAllControllers() {
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//num of controller
	m_UdpServer.write((uint8_t) __aquaduino->getNrOfControllers());

	//controller information
	Controller* controller;
	__aquaduino->resetControllerIterator();
	while (__aquaduino->getNextController(&controller) != -1) {
		m_UdpServer.write(__aquaduino->getControllerID(controller));
		m_UdpServer.write(strlen(controller->getName()));
		m_UdpServer.write(controller->getName());
		m_UdpServer.write(controller->getType());
	}
}
/*
 void GUIServer::setSerialPHConfig(uint8_t sensorId,uint8_t) {
 Sensor* sensor = __aquaduino->getSensor(sensorId);
 if (!sensor) {
 //errorcode 10 -> sensor not available
 m_UdpServer.write((uint8_t) 10);
 }
 switch (sensor->getType()) {
 case SENSOR_SERIALINPUT:
 break;
 case SENSOR_DS18S20:
 break;
 default:
 break;
 }
 }*/

void GUIServer::getClockTimers(uint8_t controllerId) {
	ClockTimerController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_CLOCKTIMER) {
		controller = (ClockTimerController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//controllerId
	m_UdpServer.write((uint8_t) controllerId);
	//num of timers
	m_UdpServer.write((uint8_t) MAX_CLOCKTIMERS);
	//num of timers per timer
	m_UdpServer.write((uint8_t) CLOCKTIMER_MAX_TIMERS);
	ClockTimer* timer;
	int i = 0;
	int j = 0;
	while (i < MAX_CLOCKTIMERS) {
		timer = controller->getClockTimer(i);
		//ClockTimerId
		m_UdpServer.write(i);
		j = 0;
		while (j < CLOCKTIMER_MAX_TIMERS) {
			m_UdpServer.write((uint8_t) timer->getHourOn(j));
			m_UdpServer.write((uint8_t) timer->getMinuteOn(j));
			m_UdpServer.write((uint8_t) timer->getHourOff(j));
			m_UdpServer.write((uint8_t) timer->getMinuteOff(j));
			j++;
		}
		m_UdpServer.write(timer->getDaysEnabled());
		m_UdpServer.write(controller->getAssignedActuatorID(i));
		i++;
	}
}
void GUIServer::setClockTimer(uint8_t controllerId) {
	ClockTimerController* controller;
	ClockTimer* timer;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_CLOCKTIMER) {
		controller = (ClockTimerController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	if (controller->getClockTimer(m_Buffer[3])) {
		timer = controller->getClockTimer(m_Buffer[3]);
	} else {
		//errorcode 11
		m_UdpServer.write((uint8_t) 11);
		return;
	}
	int j = 0;
	while (j < CLOCKTIMER_MAX_TIMERS) {
		timer->setTimer(j, m_Buffer[4 + j * CLOCKTIMER_MAX_TIMERS],
				m_Buffer[5 + j * CLOCKTIMER_MAX_TIMERS],
				m_Buffer[6 + j * CLOCKTIMER_MAX_TIMERS],
				m_Buffer[7 + j * CLOCKTIMER_MAX_TIMERS]);
		j++;

	}

	timer->setDaysEnabled(m_Buffer[4 + j * CLOCKTIMER_MAX_TIMERS]);
	//
	int8_t oldActuatorID = controller->getAssignedActuatorID(m_Buffer[3]);
	int8_t newActuatorID = m_Buffer[5 + j * CLOCKTIMER_MAX_TIMERS];
	if (newActuatorID == 255) {
		newActuatorID = -1;
	}
	Serial.print(
			"changeActuatorAssignment: [oldActuatorID][newActuatorID][controllerId]: ");
	Serial.print(oldActuatorID);
	Serial.print(" ");
	Serial.print(newActuatorID);
	Serial.print(" ");
	Serial.println(controllerId);

	changeActuatorAssignment(oldActuatorID, newActuatorID, controllerId);

	Serial.print("set clocktimer actuator to: [clocktimer][actuator]");
	Serial.print(m_Buffer[3]);
	Serial.println(newActuatorID);

	controller->assignActuatorToClockTimer(m_Buffer[3], newActuatorID);
	__aquaduino->writeConfig(controller);

	//errorcode 0
	m_UdpServer.write((uint8_t) 0);

	return;
}
////////////////////////////
// Temperature Controller
void GUIServer::getTemperatureController(uint8_t controllerId) {
	TemperatureController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_TEMPERATURE) {
		controller = (TemperatureController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//Sensor
	m_UdpServer.write(controller->getAssignedSensor());
	//TemperatureLow 16
	uint16_t tmp = controller->getRefTempLow() * 10;
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//heatingHysteresis double
	tmp = controller->getHeatingHysteresis() * 10;
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//heatingActuator
	m_UdpServer.write(controller->getHeatingActuator());
	//TemperatureHigh
	tmp = controller->getRefTempHigh() * 10;
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//coolingHysteresis
	tmp = controller->getCoolingHysteresis() * 10;
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//coolingActuaor
	m_UdpServer.write(controller->getCoolingActuator());
}
void GUIServer::setTemperatureController(uint8_t controllerId) {
	TemperatureController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_TEMPERATURE) {
		controller = (TemperatureController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	double tmp1 = *((int16_t*) &m_Buffer[3]);
	//Serial.print("low: ");
	//Serial.println(tmp1);
	tmp1 = tmp1 / 10;
	//Serial.print("/10: ");
	//Serial.println(tmp1);
	controller->setRefTempLow(tmp1);
	//Serial.print("low:");
	//Serial.println(controller->getRefTempLow());
	//
	tmp1 = *((int16_t*) &m_Buffer[5]);
	tmp1 = tmp1 / 10;
	controller->setHeatingHysteresis(tmp1);
	//
	int8_t oldActuatorID = controller->getHeatingActuator();
	int8_t newActuatorID = m_Buffer[7];
	if (newActuatorID == 255) {
		newActuatorID = -1;
	}
	changeActuatorAssignment(oldActuatorID, newActuatorID, controllerId);
	controller->assignHeatingActuator(newActuatorID);
	//
	tmp1 = *((int16_t*) &m_Buffer[8]);
	tmp1 = tmp1 / 10;
	controller->setRefTempHigh(tmp1);
	//
	tmp1 = *((int16_t*) &m_Buffer[10]);
	tmp1 = tmp1 / 10;
	controller->setCoolingHysteresis(tmp1);
	//
	oldActuatorID = controller->getCoolingActuator();
	newActuatorID = m_Buffer[12];
	if (newActuatorID == 255) {
		newActuatorID = -1;
	}
	changeActuatorAssignment(oldActuatorID, newActuatorID, controllerId);
	controller->assignCoolingActuator(newActuatorID);

	uint8_t tmp2 = m_Buffer[13];
	if (tmp2 == 255) {
		tmp2 = -1;
	}
	controller->assignSensor(tmp2);

	__aquaduino->writeConfig(controller);
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
}
void GUIServer::setTemperatureControllerName(uint8_t controllerId) {
	TemperatureController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_TEMPERATURE) {
		controller = (TemperatureController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	controller->setName((char*) &m_Buffer[3]);
	__aquaduino->writeConfig(controller);
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
}
//////////////////////
// LevelController

void GUIServer::getLevelController(uint8_t controllerId) {
	LevelController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_LEVEL) {
		controller = (LevelController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//delayHigh
	uint16_t tmp = controller->getDelayHigh();
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//delayLow
	tmp = controller->getDelayLow();
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//timeout
	tmp = controller->getTimeout();
	m_UdpServer.write((uint8_t*) &tmp, sizeof(int16_t));
	//sensor
	m_UdpServer.write(controller->getAssignedSensor());
	//actuator
	Actuator* actuator;
	__aquaduino->resetActuatorIterator();
	uint8_t actuatorId = -1;
	while (__aquaduino->getNextActuator(&actuator) != -1) {
		if (actuator->getController() == controllerId) {
			actuatorId = __aquaduino->getActuatorID(actuator);
			break;
		}

	}
	m_UdpServer.write(actuatorId);
	//state
	m_UdpServer.write(controller->getState());

}
void GUIServer::setLevelController(uint8_t controllerId) {
	LevelController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_LEVEL) {
		controller = (LevelController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	controller->setDelayHigh(*((int16_t*) &m_Buffer[3]));
	controller->setDelayLow(*((int16_t*) &m_Buffer[5]));
	controller->setTimeout(*((int16_t*) &m_Buffer[7]));

	int8_t tmp = m_Buffer[9];
	if (tmp == 255) {
		tmp = -1;
	}
	controller->assignSensor(tmp);

	//ToDo assign Actuator
	tmp = m_Buffer[10];
	if (tmp == 255) {
		tmp = -1;
	}
	int8_t oldActuatorID;
	int8_t newActuatorID = tmp;
	Serial.print("Set Level actuator: ");
	Serial.println(newActuatorID);
	Actuator* actuator;
	__aquaduino->resetActuatorIterator();
	while (__aquaduino->getNextActuator(&actuator) != -1) {
		if (actuator->getController() == controllerId) {
			actuator->setController(-1);
			__aquaduino->writeConfig(actuator);
//
			oldActuatorID = __aquaduino->getActuatorID(actuator);
			Serial.print("found old actuator: ");
			Serial.println(oldActuatorID);
			//
			break;
		}

	}
	if (newActuatorID != -1) {
		actuator = __aquaduino->getActuator(newActuatorID);
		actuator->setController(controllerId);
		__aquaduino->writeConfig(actuator);
	}
	//
	//
	__aquaduino->resetActuatorIterator();
	while (__aquaduino->getNextActuator(&actuator) != -1) {
		Serial.print(" actuator: [actuatorID][controllerId]");
		Serial.print(__aquaduino->getActuatorID(actuator));
		Serial.print(" ");
		Serial.println(actuator->getController());
		if (actuator->getController() == controllerId) {
			Serial.print("found set actuator: ");
			Serial.println(__aquaduino->getActuatorID(actuator));
			break;
		}

	}
	//
	//
	__aquaduino->writeConfig(controller);
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
}
void GUIServer::setLevelControllerName(uint8_t controllerId) {
	LevelController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_LEVEL) {
		controller = (LevelController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}

	controller->setName((char*) &m_Buffer[3]);
	__aquaduino->writeConfig(controller);
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
}
void GUIServer::resetLevelController(uint8_t controllerId) {
	LevelController* controller;
	if (__aquaduino->getController(controllerId)->getType()
			== CONTROLLER_LEVEL) {
		controller = (LevelController*) __aquaduino->getController(
				controllerId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		return;
	}
	controller->reset();
	//something to save?
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
}
/////////////////////////////////////
// Sensor
/////////////////////////////////////
//
// DS1820
void GUIServer::getDS1820Addresses() {
	/*DS18S20* sensor;
	 if (__aquaduino->getSensor(sensorId)->getType() == SENSOR_DS18S20) {
	 sensor = (DS18S20*) __aquaduino->getSensor(sensorId);
	 } else {
	 //errorcode 10
	 m_UdpServer.write((uint8_t) 10);
	 m_UdpServer.write(__aquaduino->getSensor(sensorId)->getType());
	 return;
	 }*/
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//
	OneWireHandler* onewire = __aquaduino->getOneWireHandler();
	uint8_t addr[8];
	onewire->findDevice(0, addr, 8);

	//uint8_t addr[8];
	// sensor->getAddress(addr);
	uint8_t i = 0;
	while (i < 8) {
		m_UdpServer.write(addr[i]);
		i++;
	}
}
void GUIServer::setDS1820Address(uint8_t sensorId) {
	DS18S20* sensor;
	if (__aquaduino->getSensor(sensorId)->getType() == SENSOR_DS18S20) {
		sensor = (DS18S20*) __aquaduino->getSensor(sensorId);
	} else {
		//errorcode 10
		m_UdpServer.write((uint8_t) 10);
		m_UdpServer.write(__aquaduino->getSensor(sensorId)->getType());
		return;
	}
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	uint8_t addr[8];
	uint8_t i = 0;
	while (i < 8) {
		addr[i] = m_Buffer[i + 3];
		i++;
	}
	sensor->setAddress(addr);
	__aquaduino->writeConfig(sensor);

}
