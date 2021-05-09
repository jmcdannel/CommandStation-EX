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

// Predeclare helper function
static void writeRegister(byte address, byte reg, byte value);

// Create device driver.  This function assumes that one or more PCA9685s will be installed on 
// successive I2C addresses with a contiguous range of VPINs.  For example, the first PCA9685 may
// be at address 0x40 and allocated pins 100-115.  In this case, pins 116-131 would be on another
// PCA9685 on address 0x41, pins 132-147 on address 0x42, and pins 148-163 on address 0x43.  
//
void PCA9685::create(VPIN firstVpin, int nPins, uint8_t I2CAddress) {
  PCA9685 *dev = new PCA9685(firstVpin, nPins, I2CAddress);
  addDevice(dev);
}

// Configure a port on the PCA9685.  This uses the Analogue class
bool PCA9685::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_SERVO) return false;
  if (paramCount != 4) return false;
  int activePosition = params[0];
  int inactivePosition = params[1];
  int profile = params[2];
  int initialState = params[3];
  Analogue::create(vpin, activePosition, inactivePosition, profile, initialState);
  return true;
}

// Constructor
PCA9685::PCA9685(VPIN firstVpin, int nPins, uint8_t I2CAddress) {
  _firstVpin = firstVpin;
  _nPins = min(nPins, 16);
  _I2CAddress = I2CAddress;

  // Initialise structure used for setting pulse rate
  requestBlock.setWriteParams(_I2CAddress, outputBuffer, sizeof(outputBuffer));
  I2CManager.begin();
  I2CManager.setClock(1000000); // Nominally able to run up to 1MHz on I2C
          // In reality, other devices including the Arduino will limit 
          // the clock speed to a lower rate.

  // Initialise I/O module(s) here.
    if (I2CManager.exists(_I2CAddress))
      DIAG(F("PCA9685 configured Vpins:%d-%d I2C:%x"), _firstVpin, _firstVpin+_nPins-1, _I2CAddress);
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_SLEEP | MODE1_AI);    
    writeRegister(_I2CAddress, PCA9685_PRESCALE, PRESCALE_50HZ);   // 50Hz clock, 20ms pulse period.
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_AI);
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_RESTART | MODE1_AI);
    // In theory, we should wait 500us before sending any other commands to each device, to allow
    // the PWM oscillator to get running.  However, we don't.    
}

// Device-specific initialisation
void PCA9685::_begin() {
}

// Device-specific write function.  This device is PWM, and the value written
// can be anything in the range of 0-4095 to get between 0% and 100% mark to period
// ratio.
void PCA9685::_write(VPIN vpin, int value) {
  int pin = vpin-_firstVpin;
  #ifdef DIAG_IO
  DIAG(F("PCA9685 Write Vpin:%d I2C:x%x/%d Value:%d"), (int)vpin, (int)_I2CAddress, pin, value);
  #endif
  // Wait for previous request to complete
  requestBlock.wait();
  // Set up new request.
  outputBuffer[0] = PCA9685_FIRST_SERVO + 4 * pin;
  outputBuffer[1] = 0;
  outputBuffer[2] = (value == 4095 ? 0x10 : 0);  // 4095=full on
  outputBuffer[3] = value & 0xff;
  outputBuffer[4] = value >> 8;
  I2CManager.queueRequest(&requestBlock);
}

// Display details of this device.
void PCA9685::_display() {
  DIAG(F("PCA9685 Vpins:%d-%d I2C:x%x"), (int)_firstVpin, 
    (int)_firstVpin+_nPins-1, (int)_I2CAddress);
}

// Internal helper function for this device
static void writeRegister(byte address, byte reg, byte value) {
  I2CManager.write(address, 2, reg, value);
}
