/*
 * GUIServer.cpp
 *
 *  Created on: 12.03.2014
 *      Author: Timo
 */

#include <Aquaduino.h>
#include <Framework/GUIServer.h>
#include <Arduino.h>
#include <Controller/ClockTimerController.h>

enum {
	GETVERSION = 0,
	GETALLSENSORS = 1,
	GETSENSORDATA = 2,
	SETSENSORCONFIG = 3,
	GETALLACTUATORS = 4,
	GETACTUATORDATA = 5,
	SETACTUATORDATA = 6,
	SETACTUATORCONFIG = 7,
	GETALLCONTROLLERS = 8,
	GETCLOCKTIMERS = 9
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

	Serial.print(F("UDP Packet of "));
	Serial.print(m_UdpServer.available());
	Serial.println(F(" Bytes available"));
	Serial.print(F("Got "));
	Serial.print(m_UdpServer.read(m_Buffer, sizeof(m_Buffer)));
	Serial.println(F(" Bytes"));
	return 1;
}

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
		Serial.println(m_Buffer[0]);
		Serial.print(F("Method ID: "));
		Serial.println(m_Buffer[1]);
		Serial.print(F("value: "));
		Serial.println(m_Buffer[2]);

		//switch to method
		switch (m_Buffer[1]) {
		case GETVERSION:
			m_UdpServer.write((uint8_t) 0);
			m_UdpServer.write(1);
			break;
		case GETALLSENSORS:
			getAllSensors();
			break;
		case GETSENSORDATA:
			getSensorData(m_Buffer[2]);
			break;
		case SETSENSORCONFIG:
			switch (m_Buffer[3]) {
			case 1:
			case 4:
			case 5:
				setSensorConfig(m_Buffer[2], m_Buffer[3],
						(uint8_t) m_Buffer[4]);
				break;
			case 2:
			case 3:
				setSensorConfig(m_Buffer[2], m_Buffer[3], (char*) &m_Buffer[4]);
				break;
			}
			break;
		case GETALLACTUATORS:
			getAllActuators();
			break;
		case GETACTUATORDATA:
			getActuatorData(m_Buffer[2]);
			break;
		case SETACTUATORDATA:
			setActuatorData(m_Buffer[2], m_Buffer[3], (uint8_t) m_Buffer[4],
					(uint8_t) m_Buffer[5]);
			break;
		case SETACTUATORCONFIG:
			/*1=resetOperatingHours       int     1
			 2=actuatorName              String  max 24 chars
			 3=InfluenceBitmask          int     (32 Sources)
			 4=controllerSemanticValue   int
			 5=calibrationInterval       int     days, 0=never
			 6=assignedControllerID      int
			 */
			switch (m_Buffer[3]) {
			case 1:
				Serial.println(
						"SetActuatorConfig resetOperatingHours not implemented yet");
				break;
			case 2:
				setActuatorConfig(m_Buffer[2], m_Buffer[3],
						(char*) &m_Buffer[4]);
				break;
			case 3:
				Serial.println(
						"SetActuatorConfig InfluenceBitmask not implemented yet");
				break;
			case 4:
				Serial.println(
						"SetActuatorConfig controllerSemanticValue not implemented yet");
				break;
			case 5:
				Serial.println(
						"SetActuatorConfig calibrationInterval not implemented yet");
				break;
			case 6:
				Serial.println(
						"SetActuatorConfig assignedControllerID not implemented yet");
				break;
			}
			break;
		case GETALLCONTROLLERS:
			getAllControllers();
			break;
		case GETCLOCKTIMERS:
			getClockTimers(m_Buffer[2]);
			break;
		default:
			break;
		}
		m_UdpServer.endPacket();
	}

}

union {
	unsigned long position;
	unsigned char bytes[4];
} CurrentPosition;

void GUIServer::write(uint32_t value, EthernetUDP* udpServer) {
	CurrentPosition.position = value;
	m_UdpServer.write(CurrentPosition.bytes[0]);
	m_UdpServer.write(CurrentPosition.bytes[1]);
	m_UdpServer.write(CurrentPosition.bytes[2]);
	m_UdpServer.write(CurrentPosition.bytes[3]);
}

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
		write((uint32_t) 7654321, &m_UdpServer);
		//write((uint32_t) __aquaduino->getSensorValue(sensorId) * 1000,&m_UdpServer);

		//valueMax24h:float * 1000 -> uint32
		write((uint32_t) 1234567, &m_UdpServer);
		//uint32_t valueMax24h = 65.8;
		//m_UdpServer.write((uint32_t)123);

		//valueMax24hTime:time
		//m_UdpServer.write((uint32_t) 1395867979);
		write((uint32_t) 1412928610, &m_UdpServer);

		//valueMin24h:float * 1000-> uint32
		//m_UdpServer.write((uint32_t) 4010);
		write((uint32_t) 12345, &m_UdpServer);

		//valueMin24hTime:time
		//m_UdpServer.write((uint32_t) 1395867579);
		write((uint32_t) 1412928610, &m_UdpServer);

		//lastCalibration:dateTime
		//m_UdpServer.write((uint32_t) 1395867579);
		write((uint32_t) 1412928610, &m_UdpServer);

		//operatingHours:int
		//m_UdpServer.write((uint8_t) 13);
		m_UdpServer.write((uint8_t) 10);

		//lastOperatingHoursReset
		//m_UdpServer.write((uint32_t) 1395867979);
		write((uint32_t) 1412928610, &m_UdpServer);
	} else {
		//errorcode 10 -> sensor not available
		m_UdpServer.write((uint8_t) 10);

	}

}
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

void GUIServer::setSensorConfig(uint8_t sensorId, uint8_t type, char* value) {
	Serial.print("setSensorConfig ");
	Sensor* sensor = __aquaduino->getSensor(sensorId);
	if (sensor) {
		if (type == 2) {
			sensor->setName(value);
			__aquaduino->writeConfig(sensor);
			//errorcode 0
			m_UdpServer.write((uint8_t) 0);
		}
		if (type == 3) {
			//sensorUnit
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);

	}

}
void GUIServer::setSensorConfig(uint8_t sensorId, uint8_t type, uint8_t value) {
	Serial.print("setSensorConfig Type: ");
	Serial.println(type);
	Sensor* sensor = __aquaduino->getSensor(sensorId);
	if (sensor) {
		if (type == 1) {
			//sensor->resetOperatinHours();
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

void GUIServer::setActuatorConfig(uint8_t actuatorId, uint8_t dataType,
		uint8_t data) {
	Actuator* actuator = __aquaduino->getActuator(actuatorId);
	if (actuator) {
		if (dataType == 1) {
			//actuator->resetOperatingHours();
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (dataType == 3) {
			//actuator->influenceBitmask(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (dataType == 4) {
			//actuator->controllerSemanticValue(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (dataType == 5) {
			//actuator->calibrationInterval(data);
			//errorcode 100 not implemented yet
			m_UdpServer.write((uint8_t) 100);
		}
		if (dataType == 6) {
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

void GUIServer::setActuatorConfig(uint8_t actuatorId, uint8_t dataType,
		char* data) {
	Actuator* actuator = __aquaduino->getActuator(actuatorId);
	if (actuator) {
		//Serial.println("setActuatorData: 2");
		if (dataType == 2) {
			actuator->setName(data);
		}
		__aquaduino->writeConfig(actuator);
		//errorcode 0
		m_UdpServer.write((uint8_t) 0);
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);

	}

}

void GUIServer::setActuatorData(uint8_t actuatorId, uint8_t locked, uint8_t on,
		uint8_t pwm) {
	Actuator* actuator = __aquaduino->getActuator(actuatorId);
	if (actuator) {
		//errorcode 0
		m_UdpServer.write((uint8_t) 0);
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
		Serial.print("SetActuatorData [locked][data]: ");
		Serial.print(locked);
		Serial.print(" ");
		Serial.println(on);
	} else {
		//errorcode 10 -> actuator not available
		m_UdpServer.write((uint8_t) 10);

	}

}

void GUIServer::getAllControllers() {
	//errorcode 0
	m_UdpServer.write((uint8_t) 0);
	//num of controller
	m_UdpServer.write((uint8_t) __aquaduino->getNrOfControllers());

	Serial.print("num of Controllers: ");
	Serial.println(__aquaduino->getNrOfControllers());
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
 void GUIServer::getClockTimers(uint8_t controllerId) {
 //stimmt das? oder muss ich durch iterieren?
 Controller* controller = __aquaduino->getController(controllerId)

 //num of timers
 m_UdpServer.write((uint8_t) controller->getNrOfTimers());
 //num of timers per timer
 m_UdpServer.write((uint8_t) CLOCKTIMER_MAX_TIMERS);
 ClockTimer* timer;
 controller->resetClockTimerIterator();
 while (controller->getNextClockTimer(&timer) != -1) {
 m_UdpServer.write(controller->getClockTimerID(timer));
 for (i = 0; i < CLOCKTIMER_MAX_TIMERS; i++) {
 m_UdpServer.write(timer->getHourOn(i));
 m_UdpServer.write(timer->getMinuteOn()(i));
 m_UdpServer.write(timer->getSecondOn(i));
 m_UdpServer.write(timer->getHourOff(i));
 m_UdpServer.write(timer->getMinuteOff()(i));
 m_UdpServer.write(timer->getSecondOff(i));
 }
 m_UdpServer.write(timer->isMondayEnabled());
 m_UdpServer.write(timer->isTuesdayEnabled());
 m_UdpServer.write(timer->isWednesdayEnabled());
 m_UdpServer.write(timer->isThursdayEnabled());
 m_UdpServer.write(timer->isFridayEnabled());
 m_UdpServer.write(timer->isSaturdayEnabled());
 m_UdpServer.write(timer->isSundayEnabled());
 //es fehlt der zugewiesene Actuator
 }
 }*/
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
		j=0;
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
