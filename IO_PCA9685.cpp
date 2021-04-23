/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

// REGISTER ADDRESSES
static const byte PCA9685_MODE1=0x00;      // Mode Register 
static const byte PCA9685_FIRST_SERVO=0x06;  /** low byte first servo register ON*/
static const byte PCA9685_PRESCALE=0xFE;     /** Prescale register for PWM output frequency */
// MODE1 bits
static const byte MODE1_SLEEP=0x10;   /**< Low power mode. Oscillator off */
static const byte MODE1_AI=0x20;      /**< Auto-Increment enabled */
static const byte MODE1_RESTART=0x80; /**< Restart enabled */

static const float FREQUENCY_OSCILLATOR=25000000.0; /** Accurate enough for our purposes  */
static const uint8_t PRESCALE_50HZ = (uint8_t)(((FREQUENCY_OSCILLATOR / (50.0 * 4096.0)) + 0.5) - 1);
static const uint32_t MAX_I2C_SPEED = 1000000L; // PCA9685 rated up to 1MHz I2C clock speed

// Create device driver.  This function assumes that one or more PCA9685s will be installed on 
// successive I2C addresses with a contiguous range of VPINs.  For example, the first PCA9685 may
// be at address 0x40 and allocated pins 100-115.  In this case, pins 116-131 would be on another
// PCA9685 on address 0x41, pins 132-147 on address 0x42, and pins 148-163 on address 0x43.  
//
void PCA9685::create(VPIN vpin, int nPins, uint8_t I2CAddress) {
  createInstance(vpin, nPins, I2CAddress);
}

IODevice *PCA9685::createInstance(VPIN vpin, int nPins, uint8_t I2CAddress) {
  #ifdef DIAG_IO
  DIAG(F("PCA9685 created Vpins:%d-%d"), vpin, vpin+nPins-1);
  #endif
  PCA9685 *dev = new PCA9685();
  dev->_firstID = vpin;
  dev->_nPins = min(nPins, 8*8);
  dev->_nModules = (nPins + 15) / 16;
  dev->_I2CAddress = I2CAddress;
  dev->_begin();
  addDevice(dev);
  return dev;
}

// Constructor
PCA9685::PCA9685() {}

// Device-specific initialisation
void PCA9685::_begin() {
  I2CManager.begin();
  I2CManager.setClock(1000000); // Nominally able to run up to 1MHz on I2C
          // In reality, other devices including the Arduino will limit 
          // the clock speed to a lower rate.

  // Initialise I/O module(s) here.
  for (byte module=0; module < _nPins/16; module++) {
    uint8_t address = _I2CAddress + module;
    writeRegister(address, PCA9685_MODE1, MODE1_SLEEP | MODE1_AI);    
    writeRegister(address, PCA9685_PRESCALE, PRESCALE_50HZ);   // 50Hz clock, 20ms pulse period.
    writeRegister(address, PCA9685_MODE1, MODE1_AI);
    writeRegister(address, PCA9685_MODE1, MODE1_RESTART | MODE1_AI);
    // In theory, we should wait 500us before sending any other commands to each device, to allow
    // the PWM oscillator to get running.  However, we don't.    
  }
}

// Device-specific write function.  This device is PWM, and the value written
// can be anything in the range of 0-4095 to get between 0% and 100% mark to period
// ratio.
void PCA9685::_write(VPIN vpin, int value) {
  int pin = vpin-_firstID;
  uint16_t address = _I2CAddress + pin/16;
  pin %= 16;
  #ifdef DIAG_IO
  DIAG(F("PCA9685 Write VPin:%d I2C:x%x/%d Value:%d"), (int)vpin, (int)address, pin, value);
  #endif
  uint8_t buffer[] = {(uint8_t)(PCA9685_FIRST_SERVO + 4 * pin), 
      0, 0, (uint8_t)(value & 0xff), (uint8_t)(value >> 8)};
  if (value == 4095) buffer[2] = 0x10;   // Full on
  uint8_t error = I2CManager.write(address, buffer, sizeof(buffer));
  //if (error) DIAG(F("Error I2C:%x, errCode=%d"), address, error);
  (void)error;
}

// Display details of this device.
void PCA9685::_display() {
  for (int i=0; i<_nModules; i++) {
    DIAG(F("PCA9685 VPins:%d-%d I2C:x%x"), (int)_firstID+i*16, 
      (int)min(_firstID+i*16+15,_firstID+_nPins-1), (int)(_I2CAddress+i));
  }
}

// Internal helper function for this device
void PCA9685::writeRegister(byte address, byte reg, byte value) {
  I2CManager.write(address, 2, reg, value);
}
