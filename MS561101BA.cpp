/*
MS5611-01BA.cpp - MS5611-01BA03 Driver

MS5611-01BA03 is a barometric pressure sensor whith termomether

This driver reads the device data and stores the calculated temperature (ºC), 
pressure (mbar) and altitude into public variables. The read take a time,
you can use the dataAvailable function to get if a new data is available.
See http://www.meas-spec.com/downloads/MS5611-01BA01.pdf for the device datasheet.

Tested on BeagleBone Black.
Needs the modified I2Cdev library to run.

Copyright (C) 2014 Daniel Elipe Fabiani <danelifab@gmail.com>


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include "MS561101BA.h"

/** Default constructor.
 */
MS561101BA::MS561101BA() {  
  data_available = false;
}

/** Device initialization. This function resets the device and load the C's coeficients
 * into the class registers.
 * @param address Optional address, default is 0x77
 * @return 0 = initialization succes, -1 = initialization failure
 */
int8_t MS561101BA::init(uint8_t address) {  
  _addr =  address;
  
  // reset the device to populate its internal PROM registers
  if (reset() < 0) {
    return -1;
  }
  // some safety time
  usleep(5000);
  // reads the PROM into object variables for later use
  if (readPROM() < 0) {
    return -1;
  }
  // default 2048 OSR
  oversampling = 0x06;
  
  // standar atmosferic pressure = 1013 mbar
  ref_pressure = 1013;
  
  
  
  return 0;
}

/**
 * Reads factory calibration and store it into object variables.
*/
int8_t MS561101BA::readPROM() {
  uint8_t buffer[2];
    
  for (int i=0;i<6;i++) {   
    if (I2Cdev::readBytes(_addr, MS561101BA_PROM_BASE_ADDR + (i * 2), 2, buffer) == 2){
      _C[i] = buffer[0] << 8 | buffer[1];
    }else{
      return -1; // error reading the PROM or communicating with the device
    }
    
    _C[i] = buffer[0] << 8 | buffer[1];    
  }
   
  return 0;
}
/** Read data from device and calculate temperature, pressure and altitude, stores the results into
 *  publics variables.
 * @param OSR Optional oversampling ratio, should be 256, 512, 1024, 2048 or 4096 samples. Default = 2048 
 * @return 0 = success, -1 = failure
 */
int8_t MS561101BA::readData(uint16_t OSR) {
  // see datasheet page 7 for formulas
  
  uint16_t _OSR = OSR;
  uint8_t buffer[4]; 
  data_available = false;
  
  // select OSR, if not valid OSR is send the OSR is set to default (2048)
  switch (_OSR) {
    case 256:
      oversampling = 0x00;
      break;
    case 512:
      oversampling = 0x02;
      break;
    case 1024:
      oversampling = 0x04;
      break;
    case 2048:
      oversampling = 0x06;
      break;
    case 4096:
      oversampling = 0x08;
      break;
    default:
      oversampling = 0x06;
      _OSR = 2048;
  };
  
  // initiate uncompensated pressure (D1) conversión
  if (!I2Cdev::sendCommand(_addr, MS561101BA_D1 + oversampling)) {
    return -1;
  }
  // wait for conversion
  usleep(_OSR * OSR_CONV_TIME);
  
  // send command 0 to initiate ADC read secuence
  if (!I2Cdev::sendCommand(_addr, 0)) {
    return -1;
  }
  // read ADC data
  if (I2Cdev::readData(_addr, 3, buffer) < 0) {
    return -1;
  }
  D1 = 0;
  D1 = buffer[0] << 16 | buffer[1] << 8 | buffer[2];
  
  // initiate uncompensated temperature (D2) conversión
  if (!I2Cdev::sendCommand(_addr, MS561101BA_D2 + oversampling)) {
    return -1;
  }
  // wait for conversion
  usleep(_OSR * OSR_CONV_TIME);
  // send command 0 to initiate ADC read secuence
  if (!I2Cdev::sendCommand(_addr, 0)) {
    return -1;
  }
  // read ADC data
  if (I2Cdev::readData(_addr, 3, buffer) < 0) {
    return -1;
  }
  D2 = 0;
  D2 = buffer[0] << 16 | buffer[1] << 8 | buffer[2];
    
  // calculate temperature
  dT = D2 - _C[4] * (2 << 7);
  TEMP = 2000 + dT * _C[5] / (2 << 22);
  
  // SECOND ORDER TEMPERATURE COMPENSATION
  T2 = 0;
  OFF2 = 0;
  SENS2 = 0;
  if (TEMP < 2000) {
    T2 = (dT * dT) / (2 << 30);
    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
  }
  if (TEMP < -1500) {
    OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
    SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
  }
  TEMP = TEMP - T2;
      
  // calculate temperature compensated pressure and set the public variables
  OFF = (int64_t)(uint16_t)_C[1] * (2 << 15) + ((uint16_t)_C[3] * dT) / (2 << 6);
  SENS = _C[0] * (2 << 14) + (_C[2] * dT) / (2 << 7);
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
  P = (D1 * SENS / (2 << 20) - OFF) / (2 << 14);  
  temperature = TEMP / (float)100;   
  pressure = P / (float)100;
    
  // calculate altitude and set the public variable
  altitude = ((1 - pow(pressure / ref_pressure, 0.19026)) * 288.15) / 0.00198122; // feet
  altitude = altitude * 0.3048;// conversion to meters
  
  data_available = true;
  return 0;
}

/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal C's registers with the values read from the PROM.
*/
int8_t MS561101BA::reset() {  
  if (!I2Cdev::sendCommand(_addr, MS561101BA_RESET)) {
    return -1;
  }
  return 0;    
}

/** Set the reference pressure value used for altitude calculation. 
 * @param ref_press Reference pressure. 
 */
void MS561101BA::setRefPressure(float ref_press) {
  ref_pressure = ref_press;
}

/** Gets the reference pressure value used for altitude calculation.
 * @return Reference pressure.
 */
float MS561101BA::getRefPressure() {
  return ref_pressure;
}

/** Gets if a new data is available.
 * @return true = new data is available.
 */
bool MS561101BA::dataAvailable() {
  return data_available;
}



