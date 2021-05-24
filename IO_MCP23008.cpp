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
#include "DIAG.h"
#include "I2CManager.h"

// Predeclare helper functions
static void writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t value) ;
static uint8_t readRegister(uint8_t I2CAddress, uint8_t reg);

// Constructor
MCP23008::MCP23008(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin) {
  _firstVpin = vpin;
  _nPins = min(nPins, 8);
  _I2CAddress = I2CAddress;
  _gpioInterruptPin = interruptPin;

 // Configure pin used for GPIO extender notification of change.
  if (_gpioInterruptPin >= 0)
    pinMode(_gpioInterruptPin, INPUT_PULLUP);

  // Initialise structure for reading GPIO ports
  requestBlock.setRequestParams(I2CAddress, inputBuffer, sizeof(inputBuffer), outputBuffer, sizeof(outputBuffer));
  inputBuffer[0] = REG_GPIO;

  I2CManager.begin();
  I2CManager.setClock(1000000);  // Supports fast clock
  if (I2CManager.exists(_I2CAddress))
    DIAG(F("MCP23008 I2C:%x configured Vpins:%d-%d"), _I2CAddress, _firstVpin, _firstVpin+_nPins-1);
  _portDirection = 0xff; // Defaults to Input mode
  _portPullup = 0xff; // Defaults to pullup enabled
  _portInputState = 0x00; 
  _portOutputState = 0x00; // Defaults to output zero.
  // Initialise device (in case it's warm-starting)
  I2CManager.write(_I2CAddress, 2, REG_GPIO, _portOutputState);
  I2CManager.write(_I2CAddress, 2, REG_IODIR, _portDirection);
  I2CManager.write(_I2CAddress, 2, REG_GPPU, _portPullup);
}

// Static create function
void MCP23008::create(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin) {
  MCP23008 *dev = new MCP23008(vpin, nPins, I2CAddress, interruptPin);
  if (dev) addDevice(dev);
}

// Device-specific initialisation
void MCP23008::_begin() { 
}

// Device-specific pin configuration
bool MCP23008::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount != 1) return false;
  bool pullup = params[0];
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("MCP23008 I2C:x%x Pin:%d Val:%d"), _I2CAddress, pin, pullup);
  #endif
  uint8_t mask = 1 << pin;
  if (pullup) 
    _portPullup |= mask;
  else
    _portPullup &= ~mask;
  I2CManager.write(_I2CAddress, 2, REG_GPPU, _portPullup);  
  // Read GPIO register (synchronous)
  _portInputState = readRegister(_I2CAddress, REG_GPIO);
   return true;
}


// Device-specific write function.
void MCP23008::_write(VPIN vpin, int value) {
  int pin = vpin -_firstVpin;
  #ifdef DIAG_IO
  DIAG(F("MCP23008 I2C:x%x Write Pin:%d Value:%d"), _I2CAddress, pin, value);
  #endif
  uint8_t mask = 1 << pin;
  if (value) 
    _portOutputState |= mask;
  else
    _portOutputState &= ~mask;
  writeRegister(_I2CAddress, REG_GPIO, _portOutputState);
  // Check port mode
  if (_portDirection & mask) {
    // Currently in read mode, so set to write mode
    _portDirection &= ~mask;
    writeRegister(_I2CAddress, REG_IODIR, _portDirection);
  }
  // Assume that writing to the port invalidates any cached read, so set the port counter to 0
  //  to force the port to be refreshed next time a read is issued.
  _portCounter = 0;
}

// Device-specific read function.  If port was in write mode, then set to read mode and 
// read the value of the port immediately.  Subsequent reads are done asynchronously
// within the loop() function.
int MCP23008::_read(VPIN vpin) {
  int result;
  int pin = vpin-_firstVpin;
  uint8_t mask = 1 << pin;
  if (!(_portDirection & mask)) {
    // Pin currently in write mode, so set to read mode
    _portDirection |= mask;
    writeRegister(_I2CAddress, REG_IODIR, _portDirection);
    // Read GPIO register
    _portInputState = readRegister(_I2CAddress, REG_GPIO);
  }
  if (_portInputState & mask) 
    result = 1;
  else
    result = 0;
  return result;
}

// Loop function to maintain timers associated with port read optimisation.  Decrement port counters
// periodically.  When the portCounter reaches zero, the port value is considered to be out-of-date
// and will need re-reading.
void MCP23008::_loop(unsigned long currentMicros) {
  if (requestBlock.isBusy()) return;  // Do nothing if a port read is in progress
  if (scanActive) {
    uint8_t previousState = _portInputState;
    // Scan in progress and last request completed, so retrieve port status from buffer
    if (requestBlock.status == I2C_STATUS_OK) 
      _portInputState = inputBuffer[0];
    else
      _portInputState = 0xff;
    uint8_t differences = previousState ^ _portInputState;
    #if DIAG_IO
    if (differences)
      DIAG(F("MCP23008 I2C:x%x Port Change:x%x"), (int)_I2CAddress, _portInputState);
    #else
      (void) differences;  // Suppress compiler warning.
    #endif
    scanActive = false;
  } else if (currentMicros - _lastLoopEntry > _portTickTime) {
    if (_gpioInterruptPin < 0 || digitalRead(_gpioInterruptPin) == 0) {
      // A cyclic scan is due, and either a GPIO device has pulled down its interrupt pin to
      //  signal something has changed or GPIO device interrupts are not configured
      scanActive = true;
      // Initiate read of module input register
      I2CManager.queueRequest(&requestBlock);
    }
    _lastLoopEntry = currentMicros;
  }
}

void MCP23008::_display() {
  DIAG(F("MCP23008 I2C:x%x Vpins:%d-%d"), _I2CAddress, (int)_firstVpin, 
    (int)_firstVpin+_nPins-1);
}

// Helper function to write a register
static void writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t value) {
  I2CManager.write(I2CAddress, 2, reg, value);
}

// Helper function to read a register.  Returns zero if error.
static uint8_t readRegister(uint8_t I2CAddress, uint8_t reg) {
  uint8_t buffer;
  uint8_t status = I2CManager.read(I2CAddress, &buffer, 1, &reg, 1);
  if (status == I2C_STATUS_OK) 
    return buffer;
  else
    return 0;
}

