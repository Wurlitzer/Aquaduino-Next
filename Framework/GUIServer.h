/*
 * GUIServer.h
 *
 *  Created on: 12.03.2014
 *      Author: Timo
 */

#ifndef GUISERVER_H_
#define GUISERVER_H_

#include <EthernetUdp.h>

class GUIServer {
public:
	GUIServer(uint16_t port);

	void run();

protected:
	virtual ~GUIServer();
private:
	int8_t receiveCommand();
	void changeActuatorAssignment(int8_t oldActuatorID, int8_t newActuatorID,
			int8_t controllerID);
	void getAllSensors();
	void getSensorData(uint8_t sensorId);
	void getAllActuators();
	void getActuatorData(uint8_t actuatorId);
	void getAllControllers();
	void getClockTimers(uint8_t controllerId);
	void getTemperatureController(uint8_t controllerId);
	void getLevelController(uint8_t controllerId);
	void getDS1820Addresses();
	void getOperatingTime(uint8_t actiatorId);
	void getResetOperatingTime(uint8_t actiatorId);

	void setSensorConfig(uint8_t sensorId);
	void setActuatorData(uint8_t actuatorId);
	void setActuatorConfig(uint8_t actuatorId);
	void setClockTimer(uint8_t controllerId);
	void setTemperatureController(uint8_t controllerId);
	void setTemperatureControllerName(uint8_t controllerId);
	void setLevelController(uint8_t controllerId);
	void setLevelControllerName(uint8_t controllerId);
	void resetLevelController(uint8_t controllerId);
	void setDS1820Address(uint8_t sensorId);



	void writeInt32(int32_t value);
	void writeUint32(uint32_t value);

	uint8_t m_Buffer[50];
	uint16_t m_Port;
	EthernetUDP m_UdpServer;
};

#endif /* GUISERVER_H_ */
