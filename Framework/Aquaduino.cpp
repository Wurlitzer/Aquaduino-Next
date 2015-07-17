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

/**
 * \mainpage Aquaduino
 *
 * Aquaduino
 * =========
 *
 * What is Aquaduino?
 * ------------------
 *
 * Aquaduino is an extensible open source control system framework for fish tanks
 * or other related environments. Aquaduino is published under the GPLv3 and is
 * written in C++. It is currently developed on a DFRduino Mega 2560 with DFRduino
 * Ethernet Shield. It currently supports the control of digital outputs to
 * control my relay modules, temperature readings by using a Dallas DS18S20,
 * automatic refill by using a level sensor with closing contact and a simple
 * webinterface to manage the control of the power outlets. The strength of
 * Aquaduino is its extensibility which allows developers to simply support
 * further sensors, actors or controllers related to their special needs.
 *
 * The Aquaduino framework uses Webduino for its webinterface. To provide a seamless
 * integration of actuators like relays into controlling elements the Aquaduino
 * webinterface automatically generates a configuration webinterface to assign
 * the available actuators to the controlling elements of Aquaduino. Thus the
 * actuators can easily be enabled, disabled or assigned at runtime to the
 * different control elements.
 *
 */

#include <Aquaduino.h>
#include <Controller/TemperatureController.h>
#include <Controller/LevelController.h>
#include <Controller/ClockTimerController.h>
#include <Actuators/DigitalOutput.h>
#include <Sensors/DS18S20.h>
#include <Sensors/DigitalInput.h>
#include <Sensors/SerialAtlasPH.h>
#include <Sensors/SerialAtlasEC.h>
#include <Sensors/SerialAtlasORP.h>
#include <SD.h>
#include <Time.h>
#include <EthernetUdp.h>
#include <stdlib.h>

Aquaduino* __aquaduino;

extern time_t NTPSync();
extern int freeRam();

/**
 * \brief Default Constructor
 *
 * Initializes Aquaduino with default values and then tries to read the
 * configuration using the SDConfigManager. When there are multiple
 * implementations of ConfigManager available this is the place to exchange
 * them. Finally the network is brought up.
 */
Aquaduino::Aquaduino() :
		m_IP(192, 168, 1, 222), m_Netmask(255, 255, 255, 0), m_DNSServer(192,
				168, 115, 7), m_Gateway(192, 168, 115, 7), m_NTPServer(178, 189,
				127, 147), m_Timezone(TIME_ZONE), m_NTPSyncInterval(5), m_DHCP(
				0), m_NTP(0), m_Controllers(MAX_CONTROLLERS), m_Actuators(
		MAX_ACTUATORS), m_Sensors(MAX_SENSORS), m_SCProClient(ethClient) {
	__aquaduino = this;
	m_Type = AQUADUINO;

	initPeripherals();

	m_ConfigManager = new SDConfigManager();
	readConfig(this);

	initNetwork();

	if (m_SCPro) {
		initSCPro();
	}

#ifdef INTERRUPT_DRIVEN
	Serial.println("Interrupt triggered mode enabled.");
	startTimer();
#else
	Serial.println("Software triggered mode enabled.");
#endif

	Serial.print(F("init complete --- Free Ram: "));
	Serial.println(freeRam());
	Serial.println();

}

/**
 * \brief Initialize peripherals of Arduino Board
 *
 *
 */

void Aquaduino::initPeripherals() {

//Init Serial Port
	Serial.begin(115200);

// Deselect all SPI devices!
	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);
	pinMode(10, OUTPUT);
	digitalWrite(10, HIGH);

//Initializing SD slot
	if (!SD.begin(4)) {
		Serial.println(F("No SD Card available"));
		while (1)
			;
	}

//Setting the PWM frequencies to 31.25kHz
	Serial.println(F("Initializing PWM frequency to 31.25 kHz..."));
	TCCR1A = _BV(WGM11) | _BV(WGM10);
	TCCR1B = _BV(CS10);
	TCCR2A = _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	TCCR3A = _BV(WGM31) | _BV(WGM30);
	TCCR3B = _BV(CS30);
	TCCR4A = _BV(WGM41) | _BV(WGM40);
	TCCR4B = _BV(CS40);
	TCCR5A = _BV(WGM51) | _BV(WGM50);
	TCCR5B = _BV(CS50);

	Serial.println(F("Initializing OneWire Handler..."));
	m_OneWireHandler = new OneWireHandler();
}

/**
 * \brief Initialize peripherals of Arduino Board
 *
 *
 */

void Aquaduino::initNetwork() {
	int8_t status = 0;
	/*
	 m_MAC[0] = 0xDE;
	 m_MAC[1] = 0xAD;
	 m_MAC[2] = 0xBE;
	 m_MAC[3] = 0xEF;
	 m_MAC[4] = 0xDE;
	 m_MAC[5] = 0xAD;
	 */
	if (m_DHCP) {
		Serial.println(F("Waiting for DHCP reply..."));
		status = Ethernet.begin(m_MAC);
	}
	if (!m_DHCP || !status) {
		Serial.println(F("Using static network configuration..."));
		Ethernet.begin(m_MAC, m_IP, m_DNSServer, m_Gateway, m_Netmask);
	}

	m_IP = Ethernet.localIP();
	m_DNSServer = Ethernet.dnsServerIP();
	m_Gateway = Ethernet.gatewayIP();
	m_Netmask = Ethernet.subnetMask();

	Serial.print(F("IP: "));
	Serial.println(m_IP);
	Serial.print(F("Netmask: "));
	Serial.println(m_Netmask);
	Serial.print(F("Gateway: "));
	Serial.println(m_Gateway);
	Serial.print(F("DNS Server: "));
	Serial.println(m_DNSServer);
	Serial.print(F("NTP Server: "));
	Serial.println(m_NTPServer);

//Init Time. If NTP Sync fails this will be used.
	setTime(0, 0, 0, 1, 1, 2015);

	if (isNTPEnabled()) {
		Serial.println(F("Syncing time using NTP..."));
		enableNTP();
	}

	m_GUIServer = new GUIServer(4242);
}

/**
 * \brief Setter for MAC address.
 * \param[in] mac pointer to MAC address.
 *
 * This method only stores the value in the object. It does not
 * configure the network.
 */
void Aquaduino::setMAC(uint8_t* mac) {
	for (int i = 0; i < 6; i++)
		m_MAC[i] = mac[i];
}

/**
 * \brief Getter for MAC address.
 * \param mac buffer to store the mac. Needs to be at least 6 bytes!
 *
 * \returns Configured MAC address. May be different to active MAC!
 */
void Aquaduino::getMAC(uint8_t* mac) {
	for (int i = 0; i < 6; i++)
		mac[i] = m_MAC[i];
}

/**
 * \brief Getter for IP address.
 *
 * \returns Configured IP address. May be different to active IP!
 */
IPAddress* Aquaduino::getIP() {
	return &m_IP;
}

/**
 * \brief Setter for IP address.
 * \param[in] ip pointer to IP address.
 *
 * This method only stores the value in the object. It does not
 * configure the network.
 */

void Aquaduino::setIP(IPAddress* ip) {
	m_IP = *ip;
}

/**
 * \brief Getter for netmask.
 *
 * \returns Configured netmask. May be different to active netmask!
 */

IPAddress* Aquaduino::getNetmask() {
	return &m_Netmask;
}

/**
 * \brief Setter for netmask.
 * \param[in] netmask pointer to netmask address.
 *
 * \brief This method only stores the value in the object. It does not
 * configure the network.
 */
void Aquaduino::setNetmask(IPAddress* netmask) {
	m_Netmask = *netmask;
}

/**
 * \brief Getter for gateway address.
 *
 * \returns Configured gateway address. May be different to active gateway
 * address!
 */

IPAddress* Aquaduino::getGateway() {
	return &m_Gateway;
}

/**
 * \brief Setter for gateway address.
 * \param[in] gateway pointer to gateway address.
 *
 * This method only stores the value in the object. It does not
 * configure the network.
 */

void Aquaduino::setGateway(IPAddress* gateway) {
	m_Gateway = *gateway;
}

/**
 * \brief Getter for DNS server address.
 *
 * \returns Configured DNS server address. May be different to active DNS
 * server address!
 */
IPAddress* Aquaduino::getDNS() {
	return &m_DNSServer;
}

/**
 * \brief Setter for DNS address
 * \param[in] dns pointer to DNS server address.
 *
 * This method only stores the value in the object. It does not
 * configure the network.
 */
void Aquaduino::setDNS(IPAddress* dns) {
	m_DNSServer = *dns;
}

/**
 * \brief Getter for NTP address.
 *
 * \returns Configured NTP server address.
 */
IPAddress* Aquaduino::getNTP() {
	return &m_NTPServer;
}

/**
 * \brief Setter for NTP address.
 * \param[in] ntp pointer to NTP addresss
 *
 * This method only stores the value in the object. It does not
 * trigger a NTP update.
 */
void Aquaduino::setNTP(IPAddress* ntp) {
	m_NTPServer = *ntp;
}

/**
 * \brief Getter for NTP synchronization interval.
 *
 * \returns NTP synchronization interval in minutes.
 */
uint16_t Aquaduino::getNtpSyncInterval() {
	return m_NTPSyncInterval;
}

/**
 * \brief Setter for NTP synchronization interval.
 * \param[in] syncInterval NTP synchronization interval in minutes
 *
 * This method only stores the value in the object. It does not
 * trigger a NTP update.
 */

void Aquaduino::setNtpSyncInterval(uint16_t syncInterval) {
	m_NTPSyncInterval = syncInterval;
}

void Aquaduino::setTimezone(int8_t zone) {
	this->m_Timezone = zone;
}

int8_t Aquaduino::getTimezone() {
	return this->m_Timezone;
}

/**
 * \brief Enables DHCP flag.
 *
 * Enables the DHCP flag. When this flag is set during construction time
 * a DHCP request is performed.
 */

void Aquaduino::enableDHCP() {
	m_DHCP = 1;
}

/**
 * \brief Disables DHCP flag.
 *
 * Disables the DHCP flag. When this flag is not set during construction time
 * no DHCP request is performed. Instead the IP configuration read by the
 * configuration manager is used as static configuration. The configuration
 * manager stores the values set by setIP, setNetmask, setGateway and
 * setDNS when they are updated using the configuration WebInterface.
 */
void Aquaduino::disableDHCP() {
	m_DHCP = 0;
}

/**
 * \brief Checks whether DHCP is enabled or not.
 *
 * \returns Value of the DHCP flag.
 */
int8_t Aquaduino::isDHCPEnabled() {
	return m_DHCP;
}

/**
 * \brief Enables NTP synchronization.
 *
 * Enables the NTP flag and directly performs a NTP synchronization request.
 * The NTP synchronization interval is set to the value set by
 * setNtpSyncInterval.
 */
void Aquaduino::enableNTP() {
	m_NTP = 1;
	setSyncInterval(m_NTPSyncInterval * 60);
	setSyncProvider(&::NTPSync);
}

/**
 * \brief Disables NTP synchronization.
 *
 * Disables the NTP synchronization and leaves current time untouched.
 */
void Aquaduino::disableNTP() {
	m_NTP = 0;
	setSyncInterval(m_NTPSyncInterval * 60);
	setSyncProvider(NULL);
}

/**
 * \brief Checks whether NTP synchronization is enabled or not.
 *
 * \returns Value of the NTP flag.
 */
int8_t Aquaduino::isNTPEnabled() {
	return m_NTP;
}

/**
 * \brief Sets the current time.
 * \param[in] hour
 * \param[in] minute
 * \param[in] second
 * \param[in] day
 * \param[in] month
 * \param[in] year
 *
 * Sets the current time when NTP synchronization is disabled. Otherwise no
 * update will be performed.
 */
void Aquaduino::setTime(int8_t hour, int8_t minute, int8_t second, int8_t day,
		int8_t month, int16_t year) {
	if (!m_NTP)
		::setTime(hour, minute, second, day, month, year);
}

/**
 * \brief Enables Xively.
 *
 * Enables the Xively flag. When this flag is set sensor data with valid
 * Xively channels will be send to Xively.
 */

/*void Aquaduino::initXively() {
 /*	Serial.print(F("Xively API Key: "));
 Serial.println(m_XivelyAPIKey);

 Serial.print(F("Xively Feed: "));
 Serial.println(m_XivelyFeedName);
 Serial.println(F("Xively Channels:"));
 for (uint8_t i = 0; i < getNrOfSensors(); i++) {
 Serial.print(i);
 Serial.print(":");
 Serial.println(m_XivelyChannelNames[i]);
 m_XiveleyDatastreams[i] = new XivelyDatastream(m_XivelyChannelNames[i],
 strlen(m_XivelyChannelNames[i]),
 DATASTREAM_FLOAT);
 }

 m_XivelyFeed = new XivelyFeed(atol(m_XivelyFeedName), m_XiveleyDatastreams,
 getNrOfSensors());
 }*/
void Aquaduino::initSCPro() {
	char SCProSWVersion[] = "0.0.1";

	Serial.print(F("init SCPro SW Version: "));
	Serial.println(SCProSWVersion);

	uint8_t ret = m_SCProClient.init(m_SCProServer, m_SCProServerPort,
			m_SCProServerPath, m_SCProSerial, m_SCProConnectionKey,
			SCProSWVersion);

	if (ret == 1) {
		m_SCProPutchannelRequest = new CPutchannelRequest();
		syncSec = second();
		Serial.print("Second to sync:");
		Serial.println(syncSec);

	} else {
		m_SCPro = 0;
	}

}

/**
 * \brief Adds a controller to Aquaduino.
 * \param[in] newController The controller to be added.
 *
 * Adds the controller specified by newController. The controllers are stored
 * in an ArrayList and can later be identified by their index in this
 * ArrayList. If the store operation was successful the controllers URL
 * is set to "C" followed by its index. Thus a controller stored at index 1
 * will receive the URL "C1". After the URL was set the configuration manager
 * is triggered to read the configuration of the controller.
 *
 * \returns Index of the controller in the ArrayList m_Controllers. When the
 * operation fails -1 is returned.
 */
int8_t Aquaduino::addController(Controller* newController) {

	return m_Controllers.add(newController);

}

/**
 * \brief Getter for controllers assigned to Aquaduino.
 * \param[in] idx index location.
 *
 * \returns controller object stored at position idx. Can be NULL.
 */
Controller* Aquaduino::getController(unsigned int idx) {
	return m_Controllers.get(idx);
}

/**
 * \brief Gets the index of a controller object.
 * \param[in] controller to be identified.
 *
 * \returns the index in m_Controllers if the object is stored in there. If
 * that is not the case -1 is returned.
 */
int8_t Aquaduino::getControllerID(Controller* controller) {
	return m_Controllers.findElement(controller);
}

/**
 * \brief Resets the iterator for the controllers stored in m_Controllers.
 *
 * The iterator is placed to the first slot in m_Controllers.
 */
void Aquaduino::resetControllerIterator() {
	m_Controllers.resetIterator();
}

/**
 * \brief Returns the next controller in m_Controllers.
 * \param[out] controller stores the pointer to the next controller in here.
 *
 * Since the ArrayList m_Controllers may get fragmented the ArrayList
 * provides the functionality to iterate over all available elements
 * in the ArrayList. This method delegates the call to the method of the
 * ArrayList.
 */
int8_t Aquaduino::getNextController(Controller** controller) {
	return m_Controllers.getNext(controller);
}

/**
 * \brief Getter for the number of assigned controllers.
 *
 * \returns the number of assigned controllers.
 */
unsigned char Aquaduino::getNrOfControllers() {
	return m_Controllers.getNrOfElements();
}

/**
 * \brief Adds an actuators to Aquaduino.
 * \param[in] newActuator pointer to the actuator object to be added.
 *
 * Adds the actuator specified by newActuator. The actuators are stored
 * in an ArrayList and can later be identified by their index in this
 * ArrayList. If the store operation was successful the actuators URL
 * is set to "A" followed by its index. Thus an actuator stored at index 1
 * will receive the URL "A1". After the URL was set the configuration manager
 * is triggered to read the configuration of the actuator.
 *
 * \returns Index of the actuator in the ArrayList m_Actuators. When the
 * operation fails -1 is returned.
 */
int8_t Aquaduino::addActuator(Actuator* newActuator) {

	return m_Actuators.add(newActuator);

}

/**
 * \brief Getter for actuators assigned to Aquaduino.
 * \param[in] idx index within the ArrayList m_Actuators.
 *
 * \returns actuator object stored at position idx. Can be NULL.
 */
Actuator* Aquaduino::getActuator(unsigned int idx) {
	return m_Actuators.get(idx);
}

/**
 * \brief Gets the index of an actuator object.
 * \param[in] actuator pointer to the actuator object to be found.
 *
 * \returns the index in m_Actuators if the object is stored in there. If
 * that is not the case -1 is returned.
 */
int8_t Aquaduino::getActuatorID(Actuator* actuator) {
	return m_Actuators.findElement(actuator);
}

/**
 * \brief Resets the iterator for the actuators stored in m_Actuators.
 *
 * The iterator is placed to the first slot in m_Actuators.
 */
void Aquaduino::resetActuatorIterator() {
	m_Actuators.resetIterator();
}

/**
 * \brief Returns the next actuator in m_Actuators.
 * \param[out] actuator the pointer to the actuator is stored in here.
 *
 * Since the ArrayList m_Actuators may get fragmented the ArrayList
 * provides the functionality to iterate over all available elements
 * in the ArrayList. This method delegates the call to the method of the
 * ArrayList.
 *
 * \returns the index of the next actuator.
 */
int8_t Aquaduino::getNextActuator(Actuator** actuator) {
	return m_Actuators.getNext(actuator);
}

/**
 * \brief Identifies the actuators assigned to a specific controller.
 *
 * This method iterates over all actuators and checks which actuators
 * are assigned to the controller specified by controller. The resulting
 * objects are stored in the passed array of actuator pointers with size max.
 *
 * returns the number of assigned actuators.
 */
int8_t Aquaduino::getAssignedActuators(Controller* controller,
		Actuator** actuators, int8_t max) {
	int8_t actuatorIdx = -1;
	int8_t nrOfAssignedActuators = 0;
	Actuator* currentActuator;
	int8_t controllerIdx = m_Controllers.findElement(controller);

	for (actuatorIdx = 0; actuatorIdx < MAX_ACTUATORS; actuatorIdx++) {
		currentActuator = m_Actuators.get(actuatorIdx);
		if (currentActuator
				&& currentActuator->getController() == controllerIdx) {
			if (nrOfAssignedActuators < max)
				actuators[nrOfAssignedActuators] = currentActuator;
			nrOfAssignedActuators++;
		}
	}
	return nrOfAssignedActuators;
}

/**
 * \brief Identifies the actuators assigned to a specific controller.
 * @param[in] controller The controller for which the assigned actuators shall
 *                       be identified.
 * @param[out] actuatorIDs Array to store the identified actuators.
 * @param[in] max size of the array.
 *
 * This method iterates over all actuators and checks which actuators
 * are assigned to the specified controller. The resulting
 * indices are stored in the passed array of indices with size max.
 *
 * returns the number of assigned actuators.
 */
int8_t Aquaduino::getAssignedActuatorIDs(Controller* controller,
		int8_t* actuatorIDs, int8_t max) {
	int8_t actuatorIdx = -1;
	int8_t nrOfAssignedActuators = 0;
	Actuator* currentActuator;
	int8_t controllerIdx = m_Controllers.findElement(controller);

//m_Actuators.resetIterator();
	for (actuatorIdx = 0; actuatorIdx < MAX_ACTUATORS; actuatorIdx++) {
		currentActuator = m_Actuators.get(actuatorIdx);
		if (currentActuator
				&& currentActuator->getController() == controllerIdx) {
			if (nrOfAssignedActuators < max)
				actuatorIDs[nrOfAssignedActuators] = actuatorIdx;
			nrOfAssignedActuators++;
		}
	}
	return nrOfAssignedActuators;
}

/**
 * \brief Getter for the number of assigned actuators.
 *
 * \returns the number of assigned actuators.
 */
unsigned char Aquaduino::getNrOfActuators() {
	return m_Actuators.getNrOfElements();
}

int8_t Aquaduino::addSensor(Sensor* newSensor) {

	return m_Sensors.add(newSensor);

}

Sensor* Aquaduino::getSensor(unsigned int sensor) {
	return m_Sensors[sensor];
}

int8_t Aquaduino::getSensorID(Sensor* sensor) {
	return m_Sensors.findElement(sensor);
}

void Aquaduino::resetSensorIterator() {
	m_Sensors.resetIterator();
}

int8_t Aquaduino::getNextSensor(Sensor** sensor) {
	return m_Sensors.getNext(sensor);
}

unsigned char Aquaduino::getNrOfSensors() {
	return m_Sensors.getNrOfElements();
}

double Aquaduino::getSensorValue(int8_t idx) {
	if (idx >= 0 && idx < MAX_SENSORS)
		return m_SensorReadings[idx];
	return 0;
}

OneWireHandler* Aquaduino::getOneWireHandler() {
	return m_OneWireHandler;
}

/*
 * ============================================================================
 */

/**
 * \brief Serializes the Aquaduino configuration
 * \param[out] buffer pointer to the buffer where the serialized data is going
 *                    to be stored.
 * \param[in] size Size of the buffer.
 *
 * \implements Serializable
 *
 * \returns amount of data serialized in bytes. Returns 0 if serialization
 * failed.
 */
uint16_t Aquaduino::serialize(Stream* s) {
	return 0;
}

/**
 * \brief Deserializes the Aquaduino configuration
 * \param[in] data pointer to the data where the serialized data is stored.
 * \param[in] size Size of the buffer.
 *
 * \implements Serializable
 *
 * \returns amount of data deserialized in bytes. Returns 0 if deserialization
 * failed.
 */
uint16_t Aquaduino::deserialize(Stream* s) {
	Serial.println("deserialize");
	uint8_t configVersion = 0;
	uint8_t stringLength = 0;
	uint8_t scProSerialLength = 0;
	uint8_t scProConnectionKeyLength = 0;
	uint8_t scProServerLength = 0;
	uint8_t scProServerPathLength = 0;
	uint8_t nrOfActuators = 0;
	uint8_t nrOfControllers = 0;
	uint8_t nrOfSensors = 0;
	uint16_t calculatedSize = 0;
	uint8_t i = 0;
	uint16_t size = s->available();

	memset(m_SCProConnectionKey, 0, sizeof(m_SCProConnectionKey) + 1);
	memset(m_SCProServer, 0, sizeof(m_SCProServer) + 1);

	memset(m_SCProSerial, 0, sizeof(m_SCProSerial) + 1);

	if (size >= 9) {
		configVersion = s->read();
		stringLength = s->read();
		scProSerialLength = s->read();
		scProConnectionKeyLength = s->read();
		scProServerLength = s->read();
		scProServerPathLength = s->read();
		nrOfActuators = s->read();
		nrOfControllers = s->read();
		nrOfSensors = s->read();
	}

	if (configVersion != 3) {
		Serial.print("Wrong Config File Version: ");
		Serial.print(configVersion);
		Serial.println("  should be 3");
		return 0;
	}

// See Main.java in Aquaduino-Config for calculation
	calculatedSize = 1059;

	if ((calculatedSize != size) || (stringLength != AQUADUINO_STRING_LENGTH)
			|| (scProSerialLength != SCPRO_SERIAL_LENGTH)
			|| (scProConnectionKeyLength != SCPRO_CONNECTION_KEY_LENGTH)
			|| (nrOfActuators > MAX_ACTUATORS)
			|| (nrOfControllers > MAX_CONTROLLERS)
			|| (nrOfSensors > MAX_SENSORS)) {
		Serial.println(F("Invalid configuration file size or values!"));
		Serial.print(calculatedSize);
		Serial.print(" != ");
		Serial.println(size);
	}
	//beginn deserialize
	s->readBytes((char*) m_MAC, sizeof(m_MAC));
	Serial.print(F("MAC: "));
	for (uint8_t i = 0; i < sizeof(m_MAC); i++) {
		Serial.print(m_MAC[i], HEX);
		if (i != sizeof(m_MAC) - 1)
			Serial.print(":");
	}
	Serial.println();

	s->readBytes((char*) &m_DHCP, sizeof(m_DHCP));
	Serial.print(F("DHCP: "));
	Serial.println(m_DHCP);

	Serial.print(F("IP: "));
	for (int i = 0; i < 4; i++) {
		m_IP[i] = s->read();
		Serial.print(m_IP[i]);
		if (i != 3)
			Serial.print(".");

	}
	Serial.println();

	Serial.print(F("Netmask: "));
	for (int i = 0; i < 4; i++) {
		m_Netmask[i] = s->read();
		Serial.print(m_Netmask[i]);
		if (i != 3)
			Serial.print(".");

	}
	Serial.println();

	Serial.print(F("Gateway: "));
	for (int i = 0; i < 4; i++) {
		m_Gateway[i] = s->read();
		Serial.print(m_Gateway[i]);
		if (i != 3)
			Serial.print(".");

	}
	Serial.println();

	Serial.print(F("DNS: "));
	for (int i = 0; i < 4; i++) {
		m_DNSServer[i] = s->read();
		Serial.print(m_DNSServer[i]);
		if (i != 3)
			Serial.print(".");

	}
	Serial.println();

	s->readBytes((char*) &m_NTP, 1);
	Serial.print(F("NTP: "));
	Serial.println(m_NTP);

	Serial.print(F("NTP Server: "));
	for (int i = 0; i < 4; i++) {
		m_NTPServer[i] = s->read();
		Serial.print(m_NTPServer[i]);
		if (i != 3)
			Serial.print(".");

	}
	Serial.println();

	s->readBytes((char*) &m_NTPSyncInterval, 2);
	Serial.print(F("NTP Sync Interval: "));
	Serial.println(m_NTPSyncInterval);

	s->readBytes((char*) &m_Timezone, sizeof(m_Timezone));
	Serial.print(F("Timezone: "));
	Serial.println(m_Timezone);

	for (i = 0; i < nrOfActuators; i++) {

		char name[stringLength];
		s->readBytes(name, stringLength);
		uint8_t typeId = s->read();
		uint8_t portId = s->read();
		uint8_t onValue = s->read();
		Actuator* actuator;
		int8_t idx;

		switch (typeId) {
		case 1:
			Serial.print(F("Adding I/O actuator @  Pin: "));
			Serial.print(portId);
			Serial.print(" On @ ");
			Serial.println(onValue == 1 ? 1 : 0);
			actuator = new DigitalOutput(name, onValue == 1 ? 1 : 0,
					onValue == 1 ? 0 : 1);
			((DigitalOutput*) actuator)->setPin(portId);
			break;
		default:
			actuator = NULL;
			break;
		}
		if ((actuator != NULL)
				&& (idx = __aquaduino->addActuator(actuator)) != -1) {
			readConfig(actuator);
		}
	}
	Serial.println();
	for (i = 0; i < nrOfControllers; i++) {

		char name[stringLength];
		s->readBytes(name, stringLength);
		uint8_t typeId = s->read();
		Controller* controller;
		int8_t idx;

		switch (typeId) {
		case 1:
			controller = new LevelController(name);
			break;
		case 2:
			controller = new TemperatureController(name);
			break;
		case 3:
			controller = new ClockTimerController(name);
			break;
		default:
			controller = NULL;
			break;
		}
		if ((controller != NULL)
				&& (idx = __aquaduino->addController(controller)) != -1) {
			readConfig(controller);
		}
	}
	Serial.println(__aquaduino->getNrOfControllers());
	Serial.println();
	for (i = 0; i < nrOfSensors; i++) {

		char name[stringLength];
		s->readBytes(name, stringLength);
		uint8_t typeId = s->read();
		uint8_t portId = s->read();
		uint8_t scProChannel = s->read();
		uint8_t scProChannelType = s->read();

		Sensor* sensor;
		int8_t idx;

		switch (typeId) {
		case 1:
			Serial.print(F("Adding Digitalinpiut sensor @  Pin: "));
			Serial.print(portId);
			Serial.print(" scpro: ");
			Serial.println(scProChannel);
			sensor = new DigitalInput();
			((DigitalInput*) sensor)->setPin(portId);
			((DigitalInput*) sensor)->setName(name);
			break;
		case 2:
			sensor = new DS18S20();
			Serial.print(F("Adding DS18S20 sensor @  Pin: "));
			Serial.print(portId);
			Serial.print(" scpro: ");
			Serial.println(scProChannel);
			((DS18S20*) sensor)->setPin(portId);
			((DS18S20*) sensor)->setName(name);
			m_OneWireHandler->addPin(portId);
			break;
		case 3:
			sensor = new SerialAtlasPH();
			Serial.print(F("Adding SerialAtlasPH sensor @  Serial: "));
			Serial.print(portId);
			Serial.print(" scpro: ");
			Serial.println(scProChannel);
			((SerialAtlasPH*) sensor)->setName(name);
			break;
		case 4:
			sensor = new SerialAtlasEC();
			Serial.print(F("Adding SerialAtlasEC sensor @  Serial: "));
			Serial.print(portId);
			Serial.print(" scpro: ");
			Serial.println(scProChannel);
			((SerialAtlasEC*) sensor)->setName(name);
			break;
		case 5:
			sensor = new SerialAtlasORP();
			Serial.print(F("Adding SerialAtlasORP sensor @  Serial: "));
			Serial.print(portId);
			Serial.print(" scpro: ");
			Serial.println(scProChannel);
			((SerialAtlasORP*) sensor)->setName(name);
			break;
		default:
			sensor = NULL;
			break;
		}

		if ((sensor != NULL) && (idx = __aquaduino->addSensor(sensor)) != -1) {
			readConfig(sensor);
		}
	}
	Serial.println();
	s->readBytes((char*) &m_SCPro, sizeof(m_SCPro));
	Serial.print(F("SCPro: "));
	Serial.println(m_SCPro);

	s->readBytes((char*) &m_SCProSerial, SCPRO_SERIAL_LENGTH);
	m_SCProSerial[sizeof(m_SCProSerial) - 1] = 0;
	Serial.print(F("SCPro Serial: "));
	Serial.println(m_SCProSerial);

	s->readBytes((char*) &m_SCProConnectionKey, SCPRO_CONNECTION_KEY_LENGTH);
	m_SCProConnectionKey[sizeof(m_SCProConnectionKey) - 1] = 0;
	Serial.print(F("SCPro Connection Key: "));
	Serial.println(m_SCProConnectionKey);

	s->readBytes((char*) &m_SCProServer, SCPRO_SERVER_LENGTH);
	m_SCProServer[sizeof(m_SCProServer) - 1] = 0;
	Serial.print(F("SCPro Server: "));
	Serial.println(m_SCProServer);

	//s->readBytes((char*) &m_SCProServerPort, 2);
	m_SCProServerPort = s->read() << 8;
	m_SCProServerPort += s->read();

	Serial.print(F("SCPro ServerPort: "));
	Serial.println(m_SCProServerPort);

	s->readBytes((char*) &m_SCProServerPath, SCPRO_SERVER_PATH_LENGTH);
	m_SCProServerPath[sizeof(m_SCProServerPath) - 1] = 0;
	Serial.print(F("SCPro ServerPath: "));
	Serial.println(m_SCProServerPath);

	return size;
}
/*uint16_t Aquaduino::deserialize(Stream* s) {
 Serial.println(F("deserialize"));
 uint16_t stringLength = 0;
 uint16_t scProSerialLength = 0;
 uint16_t scProConnectionKeyLength = 0;

 uint16_t nrOfActuators = 0;
 uint16_t nrOfControllers = 0;
 uint16_t nrOfSensors = 0;
 uint16_t calculatedSize = 0;
 uint16_t i = 0;
 uint16_t size = s->available();

 memset(m_SCProConnectionKey, 0, sizeof(m_SCProConnectionKey) + 1);
 memset(m_SCProSerial, 0, sizeof(m_SCProSerial) + 1);

 if (size >= 7) {
 stringLength = s->read();
 scProSerialLength = s->read();
 scProConnectionKeyLength = s->read();
 nrOfActuators = s->read();
 nrOfControllers = s->read();
 nrOfSensors = s->read();
 }

 // See Main.java in Aquaduino-Config for calculation
 calculatedSize = 7
 + ((nrOfActuators + nrOfControllers + nrOfSensors) * stringLength)
 + (3 * nrOfActuators) + nrOfControllers + (3 * nrOfSensors);
 +6 + 1 + 4 + 4 + 4 + 4 + 1 + 4 + 1 + 1 + 1 + scProSerialLength
 + scProConnectionKeyLength;

 if ((calculatedSize != size) || (stringLength != AQUADUINO_STRING_LENGTH)
 || (scProSerialLength != SCPRO_SERIAL_LENGTH)
 || (scProConnectionKeyLength != SCPRO_CONNECTION_KEY_LENGTH)
 || (nrOfActuators > MAX_ACTUATORS)
 || (nrOfControllers > MAX_CONTROLLERS)
 || (nrOfSensors > MAX_SENSORS)) {
 Serial.println(F("Invalid configuration file size! XIVELY REMOVED!"));
 Serial.print(calculatedSize);
 Serial.print(" != ");
 Serial.println(size);
 }

 for (i = 0; i < nrOfActuators; i++) {

 char name[stringLength];
 s->readBytes(name, stringLength);
 uint8_t typeId = s->read();
 uint8_t portId = s->read();
 uint8_t onValue = s->read();
 Actuator* actuator;
 int8_t idx;

 switch (typeId) {
 case 1:
 Serial.print(F("Adding I/O actuator @  Pin: "));
 Serial.print(portId);
 Serial.print(" On @ ");
 Serial.println(onValue == 1 ? 1 : 0);
 actuator = new DigitalOutput(name, onValue == 1 ? 1 : 0,
 onValue == 1 ? 0 : 1);
 ((DigitalOutput*) actuator)->setPin(portId);
 break;
 default:
 actuator = NULL;
 break;
 }

 if ((actuator != NULL)
 && (idx = __aquaduino->addActuator(actuator)) != -1) {
 readConfig(actuator);
 }
 }

 for (i = 0; i < nrOfControllers; i++) {

 char name[stringLength];
 s->readBytes(name, stringLength);
 uint8_t typeId = s->read();
 Controller* controller;
 int8_t idx;

 switch (typeId) {
 case 1:
 controller = new LevelController(name);
 break;
 case 2:
 controller = new TemperatureController(name);
 break;
 case 3:
 controller = new ClockTimerController(name);
 break;
 default:
 controller = NULL;
 break;
 }

 if ((controller != NULL)
 && (idx = __aquaduino->addController(controller)) != -1) {
 readConfig(controller);
 }
 }

 for (i = 0; i < nrOfSensors; i++) {

 char name[stringLength];
 s->readBytes(name, stringLength);
 uint8_t typeId = s->read();
 uint8_t portId = s->read();
 uint8_t scProChannel = s->read();

 Sensor* sensor;
 int8_t idx;

 switch (typeId) {
 case 1:
 Serial.print(F("Adding Digitalinpiut sensor @  Pin: "));
 Serial.print(portId);
 Serial.print(" scpro: ");
 Serial.println(scProChannel);
 sensor = new DigitalInput();
 ((DigitalInput*) sensor)->setPin(portId);
 ((DigitalInput*) sensor)->setName(name);
 break;
 case 2:
 sensor = new DS18S20();
 Serial.print(F("Adding DS18S20 sensor @  Pin: "));
 Serial.print(portId);
 Serial.print(" scpro: ");
 Serial.println(scProChannel);
 ((DS18S20*) sensor)->setPin(portId);
 ((DS18S20*) sensor)->setName(name);
 m_OneWireHandler->addPin(portId);
 break;
 case 3:
 sensor = new SerialAtlasPH();
 Serial.print(F("Adding SerialAtlasPH sensor @  Serial: "));
 Serial.print(portId);
 Serial.print(" scpro: ");
 Serial.println(scProChannel);
 ((SerialAtlasPH*) sensor)->setName(name);
 break;
 case 4:
 sensor = new SerialAtlasEC();
 Serial.print(F("Adding SerialAtlasEC sensor @  Serial: "));
 Serial.print(portId);
 Serial.print(" scpro: ");
 Serial.println(scProChannel);
 ((SerialAtlasEC*) sensor)->setName(name);
 break;
 case 5:
 sensor = new SerialAtlasORP();
 Serial.print(F("Adding SerialAtlasORP sensor @  Serial: "));
 Serial.print(portId);
 Serial.print(" scpro: ");
 Serial.println(scProChannel);
 ((SerialAtlasORP*) sensor)->setName(name);
 break;
 default:
 sensor = NULL;
 break;
 }

 if ((sensor != NULL) && (idx = __aquaduino->addSensor(sensor)) != -1) {
 //memcpy(m_XivelyChannelNames[idx], xivelyChannel,
 // xivelyChannelNameLength);
 readConfig(sensor);
 }
 }
 s->readBytes((char*) m_MAC, sizeof(m_MAC));
 Serial.print(F("MAC: "));
 for (uint8_t i = 0; i < sizeof(m_MAC); i++) {
 Serial.print(m_MAC[i], HEX);
 if (i != sizeof(m_MAC) - 1)
 Serial.print(":");
 }
 Serial.println();

 s->readBytes((char*) &m_DHCP, sizeof(m_DHCP));
 Serial.print(F("DHCP: "));
 Serial.println(m_DHCP);

 Serial.print(F("IP: "));
 for (int i = 0; i < 4; i++) {
 m_IP[i] = s->read();
 Serial.print(m_IP[i]);
 if (i != 3)
 Serial.print(".");

 }
 Serial.println();

 Serial.print(F("Netmask: "));
 for (int i = 0; i < 4; i++) {
 m_Netmask[i] = s->read();
 Serial.print(m_Netmask[i]);
 if (i != 3)
 Serial.print(".");

 }
 Serial.println();

 Serial.print(F("Gateway: "));
 for (int i = 0; i < 4; i++) {
 m_Gateway[i] = s->read();
 Serial.print(m_Gateway[i]);
 if (i != 3)
 Serial.print(".");

 }
 Serial.println();

 Serial.print(F("DNS: "));
 for (int i = 0; i < 4; i++) {
 m_DNSServer[i] = s->read();
 Serial.print(m_DNSServer[i]);
 if (i != 3)
 Serial.print(".");

 }
 Serial.println();

 s->readBytes((char*) &m_NTP, 1);
 Serial.print(F("NTP: "));
 Serial.println(m_NTP);

 Serial.print(F("NTP Server: "));
 for (int i = 0; i < 4; i++) {
 m_NTPServer[i] = s->read();
 Serial.print(m_NTPServer[i]);
 if (i != 3)
 Serial.print(".");

 }
 Serial.println();

 //ToDo: Inconsistent to Aquaduino-Config
 s->readBytes((char*) &m_NTPSyncInterval, 1);
 Serial.print(F("NTP Sync Interval: "));
 Serial.println(m_NTPSyncInterval);

 s->readBytes((char*) &m_Timezone, sizeof(m_Timezone));
 Serial.print(F("Timezone: "));
 Serial.println(m_Timezone);

 s->readBytes((char*) &m_SCPro, sizeof(m_SCPro));
 Serial.print(F("SCPro: "));
 Serial.println(m_SCPro);

 s->readBytes((char*) &m_SCProConnectionKey, SCPRO_CONNECTION_KEY_LENGTH);
 Serial.print(F("SCPro Connection Key: "));
 Serial.println(m_SCProConnectionKey);

 s->readBytes((char*) &m_SCProSerial, SCPRO_SERIAL_LENGTH);
 Serial.print(F("SCPro Serial: "));
 Serial.println(m_SCProSerial);

 return size;
 }*/

/**
 * \brief Write Aquaduino configuration
 * \param[in] aquaduino The aquaduino instance of which the configuration
 *                      shall be written.
 *
 * Delegates the call to the ConfigurationManager to write the configuration.
 *
 * \returns The number of written bytes. -1 if writing failed.
 */
int8_t Aquaduino::writeConfig(Aquaduino* aquaduino) {
	if (m_ConfigManager != NULL) {
		//ToDo:Buggy
		m_ConfigManager->writeConfig(aquaduino);
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
	}
	return 0;
}

/**
 * \brief Write Actuator configuration
 * \param[in] actuator The actuator instance of which the configuration
 *                     shall be written.
 *
 * Delegates the call to the ConfigurationManager to write the configuration.
 *
 * \returns The number of written bytes. -1 if writing failed.
 */
int8_t Aquaduino::writeConfig(Actuator* actuator) {
	if (m_ConfigManager != NULL) {
		//ToDo:Buggy
		m_ConfigManager->writeConfig(actuator);
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
	}
	return 0;
}

/**
 * \brief Write Controller configuration
 * \param[in] controller The controller instance of which the configuration
 *                       shall be written.
 *
 * Delegates the call to the ConfigurationManager to write the configuration.
 *
 * \returns The number of written bytes. -1 if writing failed.
 */
int8_t Aquaduino::writeConfig(Controller* controller) {
	if (m_ConfigManager != NULL) {
		//ToDo:Buggy
		m_ConfigManager->writeConfig(controller);
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
	}
	return 0;
}

/**
 * \brief Write Sensor configuration
 * \param[in] sensor The sensor instance of which the configuration
 *                   shall be written.
 *
 * Delegates the call to the ConfigurationManager to write the configuration.
 *
 * \returns The number of written bytes. -1 if writing failed.
 */
int8_t Aquaduino::writeConfig(Sensor* sensor) {
	if (m_ConfigManager != NULL) {
		//ToDo:Buggy
		m_ConfigManager->writeConfig(sensor);
		digitalWrite(13, HIGH);
		delay(300);
		digitalWrite(13, LOW);
	}
	return 0;
}
/**
 * \brief Write operating hours to SD Card
 * \param[in] sensor The sensor instance of which the configuration
 *                   shall be written.
 *
 * Delegates the call to the ConfigurationManager to write the configuration.
 *
 * \returns The number of written bytes. -1 if writing failed.
 */
int8_t Aquaduino::writeOperatingHours(Aquaduino* aquaduino) {
	if (m_ConfigManager != NULL) {
		m_ConfigManager->writeOpTime(aquaduino);
	}
	return 0;
}
/**
 * \brief Reads the Aquaduino configuration
 * \param[in] aquaduino The aquaduino instance of which the configuration
 *                     shall be read.
 *
 * \returns amount of data read in bytes. -1 if reading failed.
 */
int8_t Aquaduino::readConfig(Aquaduino* aquaduino) {
	IPAddress ip(192, 168, 2, 2);
	IPAddress nm(255, 255, 255, 0);

	if (m_ConfigManager != NULL) {
		Serial.println(F("Reading aquas.cfg..."));
		Serial.print(m_ConfigManager->readConfig(aquaduino));
		Serial.println(F("...Reading aquas.cfg finished."));
		Serial.print(F("Reading operating hours..."));
		Serial.print(m_ConfigManager->readOpHours(aquaduino));
		Serial.println(F("...done"));
	}
	return 0;
}

/**
 * \brief Reads Actuator configuration
 * \param[in] actuator The actuator instance of which the configuration
 *                     shall be read.
 *
 * Delegates the call to the ConfigurationManager to read the configuration.
 *
 * \returns The number of read bytes. -1 if reading failed.
 */
int8_t Aquaduino::readConfig(Actuator* actuator) {
	if (m_ConfigManager != NULL) {
		//ToDo:Buggy
		m_ConfigManager->readConfig(actuator);
	}
	return 0;
}

/**
 * \brief Reads Controller configuration
 * \param[in] controller The controller instance of which the configuration
 *                       shall be read.
 *
 * Delegates the call to the ConfigurationManager to read the configuration.
 *
 * \returns The number of read bytes. -1 if reading failed.
 */
int8_t Aquaduino::readConfig(Controller* controller) {
	if (m_ConfigManager != NULL) {
		m_ConfigManager->readConfig(controller);
	}
	return 0;
}

/**
 * \brief Reads Sensor configuration
 * \param[in] sensor The sensor instance of which the configuration
 *                   shall be read.
 *
 * Delegates the call to the ConfigurationManager to read the configuration.
 *
 * \returns The number of read bytes. -1 if reading failed.
 */
int8_t Aquaduino::readConfig(Sensor* sensor) {
	if (m_ConfigManager != NULL) {
		//ToDo:Buggy
		m_ConfigManager->readConfig(sensor);
	}
	return 0;
}

int freeRam();

/**
 * \brief Starts triggering of sensors and controller by timer interrupt
 */
void Aquaduino::startTimer() {
	TCCR5A = 0;
	TCCR5B = 0;
	TCNT5 = 0;
	TIMSK5 = 0;

	OCR5A = 25000;
	TCCR5A = _BV(WGM51) | _BV(WGM50);
	TCCR5B = _BV(CS51) | _BV(CS50) | _BV(WGM53) | _BV(WGM52);

	TIMSK5 = _BV(TOIE5);
}

void Aquaduino::readSensors() {
	int8_t sensorIdx;
	Sensor* currentSensor;

	int8_t num = __aquaduino->getNrOfSensors();
	for (sensorIdx = 0; sensorIdx < num; sensorIdx++) {
		currentSensor = m_Sensors.get(sensorIdx);
		if (currentSensor) {
			m_SensorReadings[sensorIdx] = currentSensor->read();
			if (m_SCPro) {
				m_SCProPutchannelRequest->updateValue(0, sensorIdx,
						currentSensor->getType(),
						m_SensorReadings[sensorIdx] * 1000);
			}
		}
	}
}

void Aquaduino::executeControllers() {

	int8_t controllerIdx;
	Controller* currentController;

	for (controllerIdx = 0; controllerIdx < MAX_CONTROLLERS; controllerIdx++) {
		currentController = m_Controllers.get(controllerIdx);
		if (currentController)
			currentController->run();
	}
}
/*
 int8_t Aquaduino::getTimeSynced() {
 return m_timeSynced;
 }*/

/**
 * \brief Top level run method.
 *
 * This is the top level run method. It triggers the sensor readings,
 * controller run methods and the WebServer processing. Needs to be called
 * periodically i.e. within the loop() function of the Arduino environment.
 */
void Aquaduino::run() {

	static int8_t curMin = minute();
	static int8_t curHour = hour();

#ifndef INTERRUPT_DRIVEN
	readSensors();
	executeControllers();
#endif

	if (m_SCPro && minute() != curMin && second() >= syncSec) {

		curMin = minute();
		uint16_t result;

		Serial.print(F("Sending data to SCPro... "));
		result = m_SCProClient.put(m_SCProPutchannelRequest);
		Serial.print("http Result:");
		Serial.println(result);
	}
	if (hour() != curHour) {
		curHour = hour();
		Serial.print(F("writing operating hours to SD Card... "));
		writeOperatingHours(__aquaduino);
	}

	if (m_GUIServer != NULL) {
		m_GUIServer->run();
	}
}

ISR(TIMER5_OVF_vect)
{
#ifdef INTERRUPT_DRIVEN
	__aquaduino->readSensors();
	__aquaduino->executeControllers();
#endif
}

int freeRam()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

