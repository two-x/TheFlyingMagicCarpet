// Stolen DallasTemperature library, modified to allow use of timers rather than blocking delays
#pragma once
#include <stdio.h>

#define DALLASTEMPLIBVERSION "3.8.1" // To be deprecated -> TODO remove in 4.0.0

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// set to true to include code for new and delete operators
#ifndef REQUIRESNEW
#define REQUIRESNEW false
#endif

// set to true to include code implementing alarm search functions
#ifndef REQUIRESALARMS
#define REQUIRESALARMS true
#endif

#include <inttypes.h>
#ifdef __STM32F1__
#include <OneWireSTM.h>
#else
#include <OneWire.h>
#endif

// Model IDs
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28  // also MAX31820
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B  // also MAX31850
#define DS28EA00MODEL 0x42

// Error Codes
// See https://github.com/milesburton/Arduino-Temperature-Control-Library/commit/ac1eb7f56e3894e855edc3353be4bde4aa838d41#commitcomment-75490966 for the 16bit implementation. Reverted due to microcontroller resource constraints.
#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040

#define DEVICE_FAULT_OPEN_C -254
#define DEVICE_FAULT_OPEN_F -425.199982
#define DEVICE_FAULT_OPEN_RAW -32512

#define DEVICE_FAULT_SHORTGND_C -253
#define DEVICE_FAULT_SHORTGND_F -423.399994
#define DEVICE_FAULT_SHORTGND_RAW -32384

#define DEVICE_FAULT_SHORTVDD_C -252
#define DEVICE_FAULT_SHORTVDD_F -421.599976
#define DEVICE_FAULT_SHORTVDD_RAW -32256

// For readPowerSupply on oneWire bus
// definition of nullptr for C++ < 11, using official workaround:
// http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2431.pdf
#if __cplusplus < 201103L
const class
{
public:
	template <class T>
	operator T *() const {
		return 0;
	}

	template <class C, class T>
	operator T C::*() const {
		return 0;
	}

private:
	void operator&() const;
} nullptr = {};
#endif

typedef uint8_t DeviceAddress[8];

class DallasSensor {
public:

	DallasSensor();
	DallasSensor(OneWire*);
	DallasSensor(OneWire*, uint8_t);

	void setOneWire(OneWire*);

	void setPullupPin(uint8_t);

	// initialise bus
	void begin(void);

	// returns the number of devices found on the bus
	uint8_t getDeviceCount(void);

	// returns the number of DS18xxx Family devices on bus
	uint8_t getDS18Count(void);

	// returns true if address is valid
	bool validAddress(const uint8_t*);

	// returns true if address is of the family of sensors the lib supports.
	bool validFamily(const uint8_t* deviceAddress);

	// finds an address at a given index on the bus
	bool getAddress(uint8_t*, uint8_t);

	// attempt to determine if the device at the given address is connected to the bus
	bool isConnected(const uint8_t*);

	// attempt to determine if the device at the given address is connected to the bus
	// also allows for updating the read scratchpad
	bool isConnected(const uint8_t*, uint8_t*);

	// read device's scratchpad
	bool readScratchPad(const uint8_t*, uint8_t*);

	// write device's scratchpad
	void writeScratchPad(const uint8_t*, const uint8_t*);

	// read device's power requirements
	bool readPowerSupply(const uint8_t* deviceAddress = nullptr);

	// get global resolution
	uint8_t getResolution();

	// set global resolution to 9, 10, 11, or 12 bits
	void setResolution(uint8_t);

	// returns the device resolution: 9, 10, 11, or 12 bits
	uint8_t getResolution(const uint8_t*);

	// set resolution of a device to 9, 10, 11, or 12 bits
	bool setResolution(const uint8_t*, uint8_t,
	                   bool skipGlobalBitResolutionCalculation = false);

	// sets/gets the waitForConversion flag
	void setWaitForConversion(bool);
	bool getWaitForConversion(void);

	// sets/gets the checkForConversion flag
	void setCheckForConversion(bool);
	bool getCheckForConversion(void);

	struct request_t {
		bool result;
		unsigned long timestamp;

		operator bool() {
			return result;
		}
	};

	// sends command for all devices on the bus to perform a temperature conversion
	request_t requestTemperatures(void);

	// sends command for one device to perform a temperature conversion by address
	request_t requestTemperaturesByAddress(const uint8_t*);

	// sends command for one device to perform a temperature conversion by index
	request_t requestTemperaturesByIndex(uint8_t);

	// returns temperature raw value (12 bit integer of 1/128 degrees C)
	int32_t getTemp(const uint8_t*);

	// returns temperature in degrees C
	float getTempC(const uint8_t*);

	// returns temperature in degrees F
	float getTempF(const uint8_t*);

	// Get temperature for device index (slow)
	float getTempCByIndex(uint8_t);

	// Get temperature for device index (slow)
	float getTempFByIndex(uint8_t);

	// returns true if the bus requires parasite power
	bool isParasitePowerMode(void);

	// Is a conversion complete on the wire? Only applies to the first sensor on the wire.
	bool isConversionComplete(void);

	static uint32_t microsToWaitForConversion(uint8_t);

	uint32_t microsToWaitForConversion();

	// Sends command to one device to save values from scratchpad to EEPROM by index
	// Returns true if no errors were encountered, false indicates failure
	bool saveScratchPadByIndex(uint8_t);

	// Sends command to one or more devices to save values from scratchpad to EEPROM
	// Returns true if no errors were encountered, false indicates failure
	bool saveScratchPad(const uint8_t* = nullptr);

	// Sends command to one device to recall values from EEPROM to scratchpad by index
	// Returns true if no errors were encountered, false indicates failure
	bool recallScratchPadByIndex(uint8_t);

	// Sends command to one or more devices to recall values from EEPROM to scratchpad
	// Returns true if no errors were encountered, false indicates failure
	bool recallScratchPad(const uint8_t* = nullptr);

	// Sets the autoSaveScratchPad flag
	void setAutoSaveScratchPad(bool);

	// Gets the autoSaveScratchPad flag
	bool getAutoSaveScratchPad(void);

#if REQUIRESALARMS

	typedef void AlarmHandler(const uint8_t*);

	// sets the high alarm temperature for a device
	// accepts a int8_t.  valid range is -55C - 125C
	void setHighAlarmTemp(const uint8_t*, int8_t);

	// sets the low alarm temperature for a device
	// accepts a int8_t.  valid range is -55C - 125C
	void setLowAlarmTemp(const uint8_t*, int8_t);

	// returns a int8_t with the current high alarm temperature for a device
	// in the range -55C - 125C
	int8_t getHighAlarmTemp(const uint8_t*);

	// returns a int8_t with the current low alarm temperature for a device
	// in the range -55C - 125C
	int8_t getLowAlarmTemp(const uint8_t*);

	// resets internal variables used for the alarm search
	void resetAlarmSearch(void);

	// search the wire for devices with active alarms
	bool alarmSearch(uint8_t*);

	// returns true if ia specific device has an alarm
	bool hasAlarm(const uint8_t*);

	// returns true if any device is reporting an alarm on the bus
	bool hasAlarm(void);

	// runs the alarm handler for all devices returned by alarmSearch()
	void processAlarms(void);

	// sets the alarm handler
	void setAlarmHandler(const AlarmHandler *);

	// returns true if an AlarmHandler has been set
	bool hasAlarmHandler();

#endif

	// if no alarm handler is used the two bytes can be used as user data
	// example of such usage is an ID.
	// note if device is not connected it will fail writing the data.
	// note if address cannot be found no error will be reported.
	// in short use carefully
	void setUserData(const uint8_t*, int16_t);
	void setUserDataByIndex(uint8_t, int16_t);
	int16_t getUserData(const uint8_t*);
	int16_t getUserDataByIndex(uint8_t);

	// convert from Celsius to Fahrenheit
	static float toFahrenheit(float);

	// convert from Fahrenheit to Celsius
	static float toCelsius(float);

	// convert from raw to Celsius
	static float rawToCelsius(int32_t);

	// convert from Celsius to raw
	static int16_t celsiusToRaw(float);

	// convert from raw to Fahrenheit
	static float rawToFahrenheit(int32_t);

#if REQUIRESNEW

	// initialize memory area
	void* operator new(unsigned int);

	// delete memory reference
	void operator delete(void*);

#endif

	void blockTillConversionComplete(uint8_t);
	void blockTillConversionComplete(uint8_t, unsigned long);
	void blockTillConversionComplete(uint8_t, request_t);

private:
	typedef uint8_t ScratchPad[9];

	// parasite power on or off
	bool parasite;

	// external pullup
	bool useExternalPullup;
	uint8_t pullupPin;

	// used to determine the delay amount needed to allow for the
	// temperature conversion to take place
	uint8_t bitResolution;

	// used to requestTemperature with or without delay
	bool waitForConversion;

	// used to requestTemperature to dynamically check if a conversion is complete
	bool checkForConversion;

	// used to determine if values will be saved from scratchpad to EEPROM on every scratchpad write
	bool autoSaveScratchPad;

	// count of devices on the bus
	uint8_t devices;

	// count of DS18xxx Family devices on bus
	uint8_t ds18Count;

	// Take a pointer to one wire instance
	OneWire* _wire;

	// reads scratchpad and returns the raw temperature
	int32_t calculateTemperature(const uint8_t*, uint8_t*);


	// Returns true if all bytes of scratchPad are '\0'
	bool isAllZeros(const uint8_t* const scratchPad, const size_t length = 9);

	// External pullup control
	void activateExternalPullup(void);
	void deactivateExternalPullup(void);

#if REQUIRESALARMS

	// required for alarmSearch
	uint8_t alarmSearchAddress[8];
	int8_t alarmSearchJunction;
	uint8_t alarmSearchExhausted;

	// the alarm handler function pointer
	AlarmHandler *_AlarmHandler;

#endif

};
//  End of .h, start of .c ----------------------------------------------------------

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

#define MAX_CONVERSION_TIMEOUT		750

// Alarm handler
#define NO_ALARM_HANDLER ((AlarmHandler *)0)


DallasSensor::DallasSensor() {
#if REQUIRESALARMS
	setAlarmHandler(NO_ALARM_HANDLER);
#endif
	useExternalPullup = false;
}

DallasSensor::DallasSensor(OneWire* _oneWire) : DallasSensor() {
	setOneWire(_oneWire);
}

bool DallasSensor::validFamily(const uint8_t* deviceAddress) {
	switch (deviceAddress[DSROM_FAMILY]) {
	case DS18S20MODEL:
	case DS18B20MODEL:
	case DS1822MODEL:
	case DS1825MODEL:
	case DS28EA00MODEL:
		return true;
	default:
		return false;
	}
}

/*
 * Constructs DallasSensor with strong pull-up turned on. Strong pull-up is mandated in DS18B20 datasheet for parasitic
 * power (2 wires) setup. (https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf, p. 7, section 'Powering the DS18B20').
 */
DallasSensor::DallasSensor(OneWire* _oneWire, uint8_t _pullupPin) : DallasSensor(_oneWire) {
	setPullupPin(_pullupPin);
}

void DallasSensor::setPullupPin(uint8_t _pullupPin) {
	useExternalPullup = true;
	pullupPin = _pullupPin;
	pinMode(pullupPin, OUTPUT);
	deactivateExternalPullup();
}

void DallasSensor::setOneWire(OneWire* _oneWire) {

	_wire = _oneWire;
	devices = 0;
	ds18Count = 0;
	parasite = false;
	bitResolution = 9;
	waitForConversion = true;
	checkForConversion = true;
	autoSaveScratchPad = true;

}

// initialise the bus
void DallasSensor::begin(void) {

	DeviceAddress deviceAddress;

	_wire->reset_search();
	devices = 0; // Reset the number of devices when we enumerate wire devices
	ds18Count = 0; // Reset number of DS18xxx Family devices

	while (_wire->search(deviceAddress)) {

		if (validAddress(deviceAddress)) {
			devices++;

			if (validFamily(deviceAddress)) {
				ds18Count++;

				if (!parasite && readPowerSupply(deviceAddress))
					parasite = true;

				uint8_t b = getResolution(deviceAddress);
				if (b > bitResolution) bitResolution = b;
			}
		}
	}
}

// returns the number of devices found on the bus
uint8_t DallasSensor::getDeviceCount(void) {
	return devices;
}

uint8_t DallasSensor::getDS18Count(void) {
	return ds18Count;
}

// returns true if address is valid
bool DallasSensor::validAddress(const uint8_t* deviceAddress) {
	return (_wire->crc8((uint8_t*)deviceAddress, 7) == deviceAddress[DSROM_CRC]);
}

// finds an address at a given index on the bus
// returns true if the device was found
bool DallasSensor::getAddress(uint8_t* deviceAddress, uint8_t index) {

	uint8_t depth = 0;

	_wire->reset_search();

	while (depth <= index && _wire->search(deviceAddress)) {
		if (depth == index && validAddress(deviceAddress))
			return true;
		depth++;
	}

	return false;

}

// attempt to determine if the device at the given address is connected to the bus
bool DallasSensor::isConnected(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	return isConnected(deviceAddress, scratchPad);

}

// attempt to determine if the device at the given address is connected to the bus
// also allows for updating the read scratchpad
bool DallasSensor::isConnected(const uint8_t* deviceAddress,
                                    uint8_t* scratchPad) {
	bool b = readScratchPad(deviceAddress, scratchPad);
	return b && !isAllZeros(scratchPad) && (_wire->crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

bool DallasSensor::readScratchPad(const uint8_t* deviceAddress,
                                       uint8_t* scratchPad) {

	// send the reset command and fail fast
	int b = _wire->reset();
	if (b == 0)
		return false;

	_wire->select(deviceAddress);
	_wire->write(READSCRATCH);

	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18S20: store for crc
	//         DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18S20: COUNT_REMAIN
	//         DS18B20 & DS1822: store for crc
	// byte 7: DS18S20: COUNT_PER_C
	//         DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = _wire->read();
	}

	b = _wire->reset();
	return (b == 1);
}

void DallasSensor::writeScratchPad(const uint8_t* deviceAddress,
                                        const uint8_t* scratchPad) {

	_wire->reset();
	_wire->select(deviceAddress);
	_wire->write(WRITESCRATCH);
	_wire->write(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
	_wire->write(scratchPad[LOW_ALARM_TEMP]); // low alarm temp

	// DS1820 and DS18S20 have no configuration register
	if (deviceAddress[DSROM_FAMILY] != DS18S20MODEL)
		_wire->write(scratchPad[CONFIGURATION]);

	if (autoSaveScratchPad)
		saveScratchPad(deviceAddress);
	else
		_wire->reset();
}

// returns true if parasite mode is used (2 wire)
// returns false if normal mode is used (3 wire)
// if no address is given (or nullptr) it checks if any device on the bus
// uses parasite mode.
// See issue #145
bool DallasSensor::readPowerSupply(const uint8_t* deviceAddress)
{
	bool parasiteMode = false;
	_wire->reset();
	if (deviceAddress == nullptr)
		_wire->skip();
	else
		_wire->select(deviceAddress);

	_wire->write(READPOWERSUPPLY);
	if (_wire->read_bit() == 0)
		parasiteMode = true;
	_wire->reset();
	return parasiteMode;
}

// set resolution of all devices to 9, 10, 11, or 12 bits
// if new resolution is out of range, it is constrained.
void DallasSensor::setResolution(uint8_t newResolution) {

	bitResolution = constrain(newResolution, 9, 12);
	DeviceAddress deviceAddress;
	_wire->reset_search();
	for (uint8_t i = 0; i < devices; i++) {
		if(_wire->search(deviceAddress) && validAddress(deviceAddress)) {
			setResolution(deviceAddress, bitResolution, true);
		}
	}
}

/*  PROPOSAL */

// set resolution of a device to 9, 10, 11, or 12 bits
// if new resolution is out of range, 9 bits is used.
bool DallasSensor::setResolution(const uint8_t* deviceAddress,
                                      uint8_t newResolution, bool skipGlobalBitResolutionCalculation) {

	bool success = false;

	// DS1820 and DS18S20 have no resolution configuration register
	if (deviceAddress[DSROM_FAMILY] == DS18S20MODEL) {
		success = true;
	} else {

		// handle the sensors with configuration register
		newResolution = constrain(newResolution, 9, 12);
		uint8_t newValue = 0;
		ScratchPad scratchPad;

		// we can only update the sensor if it is connected
		if (isConnected(deviceAddress, scratchPad)) {
			// MAX31850 has no resolution configuration register
			// this is also a hack as the MAX31850 Coversion time is 100ms max.
			// use a low res (~10 by spec, but 9 might work) for faster blocking read times.
			if (deviceAddress[DSROM_FAMILY] == DS1825MODEL && scratchPad[CONFIGURATION] & 0x80 ) {
				success = true;
			} else {
				switch (newResolution) {
				case 12:
					newValue = TEMP_12_BIT;
					break;
				case 11:
					newValue = TEMP_11_BIT;
					break;
				case 10:
					newValue = TEMP_10_BIT;
					break;
				case 9:
				default:
					newValue = TEMP_9_BIT;
					break;
				}

				// if it needs to be updated we write the new value
				if (scratchPad[CONFIGURATION] != newValue) {
					scratchPad[CONFIGURATION] = newValue;
					writeScratchPad(deviceAddress, scratchPad);
				}
				// done
				success = true;
			}
		}
	}

	// do we need to update the max resolution used?
	if (skipGlobalBitResolutionCalculation == false) {
		bitResolution = newResolution;
		if (devices > 1) {
			DeviceAddress deviceAddr;
			_wire->reset_search();
			for (uint8_t i = 0; i < devices; i++) {
				if (bitResolution == 12) break;
				if (_wire->search(deviceAddr) && validAddress(deviceAddr)) {
					uint8_t b = getResolution(deviceAddr);
					if (b > bitResolution) bitResolution = b;
				}
			}
		}
	}

	return success;
}


// returns the global resolution
uint8_t DallasSensor::getResolution() {
	return bitResolution;
}

// returns the current resolution of the device, 9-12
// returns 0 if device not found
uint8_t DallasSensor::getResolution(const uint8_t* deviceAddress) {

	// DS1820 and DS18S20 have no resolution configuration register
	if (deviceAddress[DSROM_FAMILY] == DS18S20MODEL)
		return 12;

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {

		// MAX31850 has no resolution configuration register
		if (deviceAddress[DSROM_FAMILY] == DS1825MODEL && scratchPad[CONFIGURATION] & 0x80)
			return 12;

		switch (scratchPad[CONFIGURATION]) {
		case TEMP_12_BIT:
			return 12;

		case TEMP_11_BIT:
			return 11;

		case TEMP_10_BIT:
			return 10;

		case TEMP_9_BIT:
			return 9;
		}
	}
	return 0;

}


// sets the value of the waitForConversion flag
// TRUE : function requestTemperature() etc returns when conversion is ready
// FALSE: function requestTemperature() etc returns immediately (USE WITH CARE!!)
//        (1) programmer has to check if the needed delay has passed
//        (2) but the application can do meaningful things in that time
void DallasSensor::setWaitForConversion(bool flag) {
	waitForConversion = flag;
}

// gets the value of the waitForConversion flag
bool DallasSensor::getWaitForConversion() {
	return waitForConversion;
}

// sets the value of the checkForConversion flag
// TRUE : function requestTemperature() etc will 'listen' to an IC to determine whether a conversion is complete
// FALSE: function requestTemperature() etc will wait a set time (worst case scenario) for a conversion to complete
void DallasSensor::setCheckForConversion(bool flag) {
	checkForConversion = flag;
}

// gets the value of the waitForConversion flag
bool DallasSensor::getCheckForConversion() {
	return checkForConversion;
}

bool DallasSensor::isConversionComplete() {
	uint8_t b = _wire->read_bit();
	return (b == 1);
}

// sends command for all devices on the bus to perform a temperature conversion
DallasSensor::request_t DallasSensor::requestTemperatures() {
	DallasSensor::request_t req = {};
	req.result = true;

    // int64_t check1 = esp_timer_get_time();  // Soren
	_wire->reset();
    // int64_t check2 = esp_timer_get_time();
	// std::cout << "Temp rst:" << check2-check1;  // 1000us
	_wire->skip();
    // check1 = esp_timer_get_time();
	// std::cout << " skp:" << check1-check2;  // 1150 us
	_wire->write(STARTCONVO, parasite);
    // check2 = esp_timer_get_time();
	// std::cout << " cnv:" << check2-check1 << std::endl;  // 600 us
    // waitForConversion = false;  // Soren

	// ASYNC mode?
	req.timestamp = millis();
	if (!waitForConversion)
		return req;
	blockTillConversionComplete(bitResolution, req.timestamp);
	return req;
}

// sends command for one device to perform a temperature by address
// returns FALSE if device is disconnected
// returns TRUE  otherwise
DallasSensor::request_t DallasSensor::requestTemperaturesByAddress(const uint8_t* deviceAddress) {
	DallasSensor::request_t req = {};
	uint8_t bitResolution = getResolution(deviceAddress);
	if (bitResolution == 0) {
		req.result = false;
		return req; //Device disconnected
	}

    // int64_t check1 = esp_timer_get_time();  // Soren
	_wire->reset();
    // int64_t check2 = esp_timer_get_time();
	// std::cout << "Temp rst:" << check2-check1;  // 1000us
	_wire->select(deviceAddress);
    // check1 = esp_timer_get_time();
	// std::cout << " sel:" << check1-check2;  // 5400 us
	_wire->write(STARTCONVO, parasite);
    // check2 = esp_timer_get_time();
	// std::cout << " cnv:" << check2-check1 << std::endl;  // 600 us
    // waitForConversion = false;  // Soren

	req.timestamp = millis();
	// ASYNC mode?
	req.result = true;


	if (!waitForConversion)
		return req;

	blockTillConversionComplete(bitResolution, req.timestamp);

	return req;

}

// Continue to check if the IC has responded with a temperature
void DallasSensor::blockTillConversionComplete(uint8_t bitResolution, unsigned long start) {
	if (checkForConversion && !parasite) {
		while (!isConversionComplete() && (millis() - start <  MAX_CONVERSION_TIMEOUT))
			yield();
	} else {
		uint32_t delms = microsToWaitForConversion(bitResolution);
		activateExternalPullup();
		delay(delms);
		deactivateExternalPullup();
	}

}

// Continue to check if the IC has responded with a temperature
void DallasSensor::blockTillConversionComplete(uint8_t bitResolution) {
	unsigned long start = millis();
	blockTillConversionComplete(bitResolution, start);
}

// Continue to check if the IC has responded with a temperature
void DallasSensor::blockTillConversionComplete(uint8_t bitResolution, DallasSensor::request_t req) {
	if (req.result)
		blockTillConversionComplete(bitResolution, req.timestamp);
}

// returns number of milliseconds to wait till conversion is complete (based on IC datasheet)
uint32_t DallasSensor::microsToWaitForConversion(uint8_t bitResolution) {

	switch (bitResolution) {
	case 9:
		return 94000;
	case 10:
		return 188000;
	case 11:
		return 375000;
	default:
		return 750000;
	}

}

// returns number of milliseconds to wait till conversion is complete (based on IC datasheet)
uint32_t DallasSensor::microsToWaitForConversion() {
	return microsToWaitForConversion(bitResolution);
}

// Sends command to one device to save values from scratchpad to EEPROM by index
// Returns true if no errors were encountered, false indicates failure
bool DallasSensor::saveScratchPadByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;
	if (!getAddress(deviceAddress, deviceIndex)) return false;

	return saveScratchPad(deviceAddress);

}

// Sends command to one or more devices to save values from scratchpad to EEPROM
// If optional argument deviceAddress is omitted the command is send to all devices
// Returns true if no errors were encountered, false indicates failure
bool DallasSensor::saveScratchPad(const uint8_t* deviceAddress) {

	if (_wire->reset() == 0)
		return false;

	if (deviceAddress == nullptr)
		_wire->skip();
	else
		_wire->select(deviceAddress);

	_wire->write(COPYSCRATCH, parasite);

	// Specification: NV Write Cycle Time is typically 2ms, max 10ms
	// Waiting 20ms to allow for sensors that take longer in practice
	if (!parasite) {
		delay(20);
	} else {
		activateExternalPullup();
		delay(20);
		deactivateExternalPullup();
	}

	return _wire->reset() == 1;

}

// Sends command to one device to recall values from EEPROM to scratchpad by index
// Returns true if no errors were encountered, false indicates failure
bool DallasSensor::recallScratchPadByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;
	if (!getAddress(deviceAddress, deviceIndex)) return false;

	return recallScratchPad(deviceAddress);

}

// Sends command to one or more devices to recall values from EEPROM to scratchpad
// If optional argument deviceAddress is omitted the command is send to all devices
// Returns true if no errors were encountered, false indicates failure
bool DallasSensor::recallScratchPad(const uint8_t* deviceAddress) {

	if (_wire->reset() == 0)
		return false;

	if (deviceAddress == nullptr)
		_wire->skip();
	else
		_wire->select(deviceAddress);

	_wire->write(RECALLSCRATCH, parasite);

	// Specification: Strong pullup only needed when writing to EEPROM (and temp conversion)
	unsigned long start = millis();
	while (_wire->read_bit() == 0) {
		// Datasheet doesn't specify typical/max duration, testing reveals typically within 1ms
		if (millis() - start > 20) return false;
		yield();
	}

	return _wire->reset() == 1;

}

// Sets the autoSaveScratchPad flag
void DallasSensor::setAutoSaveScratchPad(bool flag) {
	autoSaveScratchPad = flag;
}

// Gets the autoSaveScratchPad flag
bool DallasSensor::getAutoSaveScratchPad() {
	return autoSaveScratchPad;
}

void DallasSensor::activateExternalPullup() {
	if (useExternalPullup)
		digitalWrite(pullupPin, LOW);
}

void DallasSensor::deactivateExternalPullup() {
	if (useExternalPullup)
		digitalWrite(pullupPin, HIGH);
}

// sends command for one device to perform a temp conversion by index
DallasSensor::request_t DallasSensor::requestTemperaturesByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;
	getAddress(deviceAddress, deviceIndex);

	return requestTemperaturesByAddress(deviceAddress);

}

// Fetch temperature for device index
float DallasSensor::getTempCByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;
	if (!getAddress(deviceAddress, deviceIndex)) {
		return DEVICE_DISCONNECTED_C;
	}
	return getTempC((uint8_t*) deviceAddress);
}

// Fetch temperature for device index
float DallasSensor::getTempFByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;

	if (!getAddress(deviceAddress, deviceIndex)) {
		return DEVICE_DISCONNECTED_F;
	}

	return getTempF((uint8_t*) deviceAddress);

}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int32_t DallasSensor::calculateTemperature(const uint8_t* deviceAddress,
                                                uint8_t* scratchPad) {

	int32_t fpTemperature = 0;

	// looking thru the spec sheets of all supported devices, bit 15 is always the signing bit
	// Detected if signed
	int32_t neg = 0x0;
	if (scratchPad[TEMP_MSB] & 0x80)
		neg = 0xFFF80000;

	// detect MAX31850
	// The temp range on a MAX31850 can far exceed other models, causing an overrun @ 256C
	// Based on the spec sheets for the MAX31850, bit 7 is always 1
	// Whereas the DS1825 bit 7 is always 0
	// DS1825   - https://datasheets.maximintegrated.com/en/ds/DS1825.pdf
	// MAX31850 - https://datasheets.maximintegrated.com/en/ds/MAX31850-MAX31851.pdf

	if (deviceAddress[DSROM_FAMILY] == DS1825MODEL && scratchPad[CONFIGURATION] & 0x80 ) {
		//Serial.print("  Detected MAX31850");
		if (scratchPad[TEMP_LSB] & 1) { // Fault Detected
			if (scratchPad[HIGH_ALARM_TEMP] & 1) {
				//Serial.println("open detected");
				return DEVICE_FAULT_OPEN_RAW;
			}
			else if (scratchPad[HIGH_ALARM_TEMP] >> 1 & 1) {
				//Serial.println("short to ground detected");
				return DEVICE_FAULT_SHORTGND_RAW;
			}
			else if (scratchPad[HIGH_ALARM_TEMP] >> 2 & 1) {
				//Serial.println("short to Vdd detected");
				return DEVICE_FAULT_SHORTVDD_RAW;
			}
			else {
				// We don't know why there's a fault, exit with disconnect value
				return DEVICE_DISCONNECTED_RAW;
			}
		}
		// We must mask out bit 1 (reserved) and 0 (fault) on TEMP_LSB
		fpTemperature = (((int32_t) scratchPad[TEMP_MSB]) << 11)
		                | (((int32_t) scratchPad[TEMP_LSB] & 0xFC) << 3)
		                | neg;
	} else {
		fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11)
		                | (((int16_t) scratchPad[TEMP_LSB]) << 3)
		                | neg;
	}

	/*
	 DS1820 and DS18S20 have a 9-bit temperature register.

	 Resolutions greater than 9-bit can be calculated using the data from
	 the temperature, and COUNT REMAIN and COUNT PER °C registers in the
	 scratchpad.  The resolution of the calculation depends on the model.

	 While the COUNT PER °C register is hard-wired to 16 (10h) in a
	 DS18S20, it changes with temperature in DS1820.

	 After reading the scratchpad, the TEMP_READ value is obtained by
	 truncating the 0.5°C bit (bit 0) from the temperature data. The
	 extended resolution temperature can then be calculated using the
	 following equation:

	                                  COUNT_PER_C - COUNT_REMAIN
	 TEMPERATURE = TEMP_READ - 0.25 + --------------------------
	                                         COUNT_PER_C

	 Hagai Shatz simplified this to integer arithmetic for a 12 bits
	 value for a DS18S20, and James Cameron added legacy DS1820 support.

	 See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
	 */

	if ((deviceAddress[DSROM_FAMILY] == DS18S20MODEL) && (scratchPad[COUNT_PER_C] != 0)) {
		fpTemperature = ((fpTemperature & 0xfff0) << 3) - 32
		                + (((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7)
		                   / scratchPad[COUNT_PER_C]);
	}

	return fpTemperature;
}

// returns temperature in 1/128 degrees C or DEVICE_DISCONNECTED_RAW if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_RAW is defined in
// DallasSensor.h. It is a large negative number outside the
// operating range of the device
int32_t DallasSensor::getTemp(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad))
		return calculateTemperature(deviceAddress, scratchPad);
	return DEVICE_DISCONNECTED_RAW;

}

// returns temperature in degrees C or DEVICE_DISCONNECTED_C if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_C is defined in
// DallasSensor.h. It is a large negative number outside the
// operating range of the device
float DallasSensor::getTempC(const uint8_t* deviceAddress) {
	return rawToCelsius(getTemp(deviceAddress));
}

// returns temperature in degrees F or DEVICE_DISCONNECTED_F if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_F is defined in
// DallasSensor.h. It is a large negative number outside the
// operating range of the device
float DallasSensor::getTempF(const uint8_t* deviceAddress) {
	return rawToFahrenheit(getTemp(deviceAddress));
}

// returns true if the bus requires parasite power
bool DallasSensor::isParasitePowerMode(void) {
	return parasite;
}

// IF alarm is not used one can store a 16 bit int of userdata in the alarm
// registers. E.g. an ID of the sensor.
// See github issue #29

// note if device is not connected it will fail writing the data.
void DallasSensor::setUserData(const uint8_t* deviceAddress,
                                    int16_t data) {
	// return when stored value == new value
	if (getUserData(deviceAddress) == data)
		return;

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {
		scratchPad[HIGH_ALARM_TEMP] = data >> 8;
		scratchPad[LOW_ALARM_TEMP] = data & 255;
		writeScratchPad(deviceAddress, scratchPad);
	}
}

int16_t DallasSensor::getUserData(const uint8_t* deviceAddress) {
	int16_t data = 0;
	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {
		data = scratchPad[HIGH_ALARM_TEMP] << 8;
		data += scratchPad[LOW_ALARM_TEMP];
	}
	return data;
}

// note If address cannot be found no error will be reported.
int16_t DallasSensor::getUserDataByIndex(uint8_t deviceIndex) {
	DeviceAddress deviceAddress;
	getAddress(deviceAddress, deviceIndex);
	return getUserData((uint8_t*) deviceAddress);
}

void DallasSensor::setUserDataByIndex(uint8_t deviceIndex, int16_t data) {
	DeviceAddress deviceAddress;
	getAddress(deviceAddress, deviceIndex);
	setUserData((uint8_t*) deviceAddress, data);
}

// Convert float Celsius to Fahrenheit
float DallasSensor::toFahrenheit(float celsius) {
	return (celsius * 1.8f) + 32.0f;
}

// Convert float Fahrenheit to Celsius
float DallasSensor::toCelsius(float fahrenheit) {
	return (fahrenheit - 32.0f) * 0.555555556f;
}

// convert from raw to Celsius
float DallasSensor::rawToCelsius(int32_t raw) {

	if (raw <= DEVICE_DISCONNECTED_RAW)
		return DEVICE_DISCONNECTED_C;
	// C = RAW/128
	return (float) raw * 0.0078125f;

}

// Convert from Celsius to raw returns temperature in raw integer format.
// The rounding error in the conversion is smaller than 0.01°C
// where the resolution of the sensor is at best 0.0625°C (in 12 bit mode).
// Rounding error can be verified by running:
//  for (float t=-55.; t<125.; t+=0.01)
//  {
//    Serial.println( DallasSensor::rawToCelsius(DallasSensor::celsiusToRaw(t))-t, 4 );
//  }
int16_t DallasSensor::celsiusToRaw(float celsius) {

	return static_cast<uint16_t>( celsius * 128.f );
}

// convert from raw to Fahrenheit
float DallasSensor::rawToFahrenheit(int32_t raw) {

	if (raw <= DEVICE_DISCONNECTED_RAW)
		return DEVICE_DISCONNECTED_F;
	// C = RAW/128
	// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
	return ((float) raw * 0.0140625f) + 32.0f;

}

// Returns true if all bytes of scratchPad are '\0'
bool DallasSensor::isAllZeros(const uint8_t * const scratchPad, const size_t length) {
	for (size_t i = 0; i < length; i++) {
		if (scratchPad[i] != 0) {
			return false;
		}
	}

	return true;
}

#if REQUIRESALARMS

/*

 ALARMS:

 TH and TL Register Format

 BIT 7 BIT 6 BIT 5 BIT 4 BIT 3 BIT 2 BIT 1 BIT 0
 S    2^6   2^5   2^4   2^3   2^2   2^1   2^0

 Only bits 11 through 4 of the temperature register are used
 in the TH and TL comparison since TH and TL are 8-bit
 registers. If the measured temperature is lower than or equal
 to TL or higher than or equal to TH, an alarm condition exists
 and an alarm flag is set inside the DS18B20. This flag is
 updated after every temperature measurement; therefore, if the
 alarm condition goes away, the flag will be turned off after
 the next temperature conversion.

 */

// sets the high alarm temperature for a device in degrees Celsius
// accepts a float, but the alarm resolution will ignore anything
// after a decimal point.  valid range is -55C - 125C
void DallasSensor::setHighAlarmTemp(const uint8_t* deviceAddress,
                                         int8_t celsius) {

	// return when stored value == new value
	if (getHighAlarmTemp(deviceAddress) == celsius)
		return;

	// make sure the alarm temperature is within the device's range
	if (celsius > 125)
		celsius = 125;
	else if (celsius < -55)
		celsius = -55;

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {
		scratchPad[HIGH_ALARM_TEMP] = (uint8_t) celsius;
		writeScratchPad(deviceAddress, scratchPad);
	}

}

// sets the low alarm temperature for a device in degrees Celsius
// accepts a float, but the alarm resolution will ignore anything
// after a decimal point.  valid range is -55C - 125C
void DallasSensor::setLowAlarmTemp(const uint8_t* deviceAddress,
                                        int8_t celsius) {

	// return when stored value == new value
	if (getLowAlarmTemp(deviceAddress) == celsius)
		return;

	// make sure the alarm temperature is within the device's range
	if (celsius > 125)
		celsius = 125;
	else if (celsius < -55)
		celsius = -55;

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {
		scratchPad[LOW_ALARM_TEMP] = (uint8_t) celsius;
		writeScratchPad(deviceAddress, scratchPad);
	}

}

// returns a int8_t with the current high alarm temperature or
// DEVICE_DISCONNECTED for an address
int8_t DallasSensor::getHighAlarmTemp(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad))
		return (int8_t) scratchPad[HIGH_ALARM_TEMP];
	return DEVICE_DISCONNECTED_C;

}

// returns a int8_t with the current low alarm temperature or
// DEVICE_DISCONNECTED for an address
int8_t DallasSensor::getLowAlarmTemp(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad))
		return (int8_t) scratchPad[LOW_ALARM_TEMP];
	return DEVICE_DISCONNECTED_C;

}

// resets internal variables used for the alarm search
void DallasSensor::resetAlarmSearch() {

	alarmSearchJunction = -1;
	alarmSearchExhausted = 0;
	for (uint8_t i = 0; i < 7; i++) {
		alarmSearchAddress[i] = 0;
	}

}

// This is a modified version of the OneWire::search method.
//
// Also added the OneWire search fix documented here:
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295
//
// Perform an alarm search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use
// DallasSensor::resetAlarmSearch() to start over.
bool DallasSensor::alarmSearch(uint8_t* newAddr) {

	uint8_t i;
	int8_t lastJunction = -1;
	uint8_t done = 1;

	if (alarmSearchExhausted)
		return false;
	if (!_wire->reset())
		return false;

	// send the alarm search command
	_wire->write(0xEC, 0);

	for (i = 0; i < 64; i++) {

		uint8_t a = _wire->read_bit();
		uint8_t nota = _wire->read_bit();
		uint8_t ibyte = i / 8;
		uint8_t ibit = 1 << (i & 7);

		// I don't think this should happen, this means nothing responded, but maybe if
		// something vanishes during the search it will come up.
		if (a && nota)
			return false;

		if (!a && !nota) {
			if (i == alarmSearchJunction) {
				// this is our time to decide differently, we went zero last time, go one.
				a = 1;
				alarmSearchJunction = lastJunction;
			} else if (i < alarmSearchJunction) {

				// take whatever we took last time, look in address
				if (alarmSearchAddress[ibyte] & ibit) {
					a = 1;
				} else {
					// Only 0s count as pending junctions, we've already exhausted the 0 side of 1s
					a = 0;
					done = 0;
					lastJunction = i;
				}
			} else {
				// we are blazing new tree, take the 0
				a = 0;
				alarmSearchJunction = i;
				done = 0;
			}
			// OneWire search fix
			// See: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295
		}

		if (a)
			alarmSearchAddress[ibyte] |= ibit;
		else
			alarmSearchAddress[ibyte] &= ~ibit;

		_wire->write_bit(a);
	}

	if (done)
		alarmSearchExhausted = 1;
	for (i = 0; i < 8; i++)
		newAddr[i] = alarmSearchAddress[i];
	return true;

}

// returns true if device address might have an alarm condition
// (only an alarm search can verify this)
bool DallasSensor::hasAlarm(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {

		int8_t temp = calculateTemperature(deviceAddress, scratchPad) >> 7;

		// check low alarm
		if (temp <= (int8_t) scratchPad[LOW_ALARM_TEMP])
			return true;

		// check high alarm
		if (temp >= (int8_t) scratchPad[HIGH_ALARM_TEMP])
			return true;
	}

	// no alarm
	return false;

}

// returns true if any device is reporting an alarm condition on the bus
bool DallasSensor::hasAlarm(void) {

	DeviceAddress deviceAddress;
	resetAlarmSearch();
	return alarmSearch(deviceAddress);
}

// runs the alarm handler for all devices returned by alarmSearch()
// unless there no _AlarmHandler exist.
void DallasSensor::processAlarms(void) {

	if (!hasAlarmHandler()) {
		return;
	}

	resetAlarmSearch();
	DeviceAddress alarmAddr;

	while (alarmSearch(alarmAddr)) {
		if (validAddress(alarmAddr)) {
			_AlarmHandler(alarmAddr);
		}
	}
}

// sets the alarm handler
void DallasSensor::setAlarmHandler(const AlarmHandler *handler) {
	_AlarmHandler = handler;
}

// checks if AlarmHandler has been set.
bool DallasSensor::hasAlarmHandler()
{
	return _AlarmHandler != NO_ALARM_HANDLER;
}

#endif

#if REQUIRESNEW

// MnetCS - Allocates memory for DallasSensor. Allows us to instance a new object
void* DallasSensor::operator new(unsigned int size) { // Implicit NSS obj size

	void * p;// void pointer
	p = malloc(size);// Allocate memory
	memset((DallasSensor*)p, 0, size); // Initialise memory

	//!!! CANT EXPLICITLY CALL CONSTRUCTOR - workaround by using an init() methodR - workaround by using an init() method
	return (DallasSensor*) p;// Cast blank region to NSS pointer
}

// MnetCS 2009 -  Free the memory used by this instance
void DallasSensor::operator delete(void* p) {

	DallasSensor* pNss = (DallasSensor*) p; // Cast to NSS pointer
	pNss->~DallasSensor();// Destruct the object

	free(p);// Free the memory
}
#endif