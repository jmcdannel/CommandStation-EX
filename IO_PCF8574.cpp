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

// Define symbol to enable PCF8574 input port value caching (reduce I2C traffic).
#define PCF8574_OPTIMISE

// Constructor
PCF8574::PCF8574(VPIN vpin, int nPins, uint8_t I2CAddress) {
  _firstVpin = vpin;
  _nPins = min(nPins, 8);
  _I2CAddress = I2CAddress;

  // Initialise structure for reading input register
  requestBlock.setReadParams(I2CAddress, inputBuffer, sizeof(inputBuffer));

  I2CManager.begin();
  I2CManager.setClock(100000);  // Only supports slow clock by default

  if (I2CManager.exists(_I2CAddress))
    DIAG(F("PCF8574 created Vpins:%d-%d I2C:%x"), _firstVpin, _firstVpin+nPins-1, _I2CAddress);
  _portInputState = 0x00;
  _portOutputState = 0x00; // Defaults to output zero.
}

// Static create method for one module.
void PCF8574::create(VPIN vpin, int nPins, uint8_t I2CAddress) {
  PCF8574 *dev = new PCF8574(vpin, nPins, I2CAddress);
  addDevice(dev);
}

// Device-specific initialisation
void PCF8574::_begin() {
}

// Device-specific pin configuration function.  The PCF8574 will not work in input mode
// unless a pullup is configured.  So return false (fail) if anything else requested.
bool PCF8574::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  (void)vpin;   // Suppress compiler warning
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount == 1) {
    bool pullup = params[0];
    if (pullup) return true;
  }
  return false;
}

// Device-specific write function.
void PCF8574::_write(VPIN vpin, int value) {
  int pin = vpin -_firstVpin;
  #ifdef DIAG_IO
  DIAG(F("PCF8574::_write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress, (int)vpin, value);
  #endif
  uint8_t mask = 1 << pin;
  if (value) 
    _portOutputState |= mask;
  else
    _portOutputState &= ~mask;
  I2CManager.write(_I2CAddress, &_portOutputState, 1);
}

// Device-specific read function.  Returns the last input value scanned.
int PCF8574::_read(VPIN vpin) {
  int result;
  int pin = vpin-_firstVpin;
  uint8_t mask = 1 << pin;
  // To enable the pin to be read, write a '1' to it first.  The connected
  // equipment should pull the input down to ground.
  if (!(_portOutputState & mask)) {
    // Pin currently driven to zero, so set to one first and then read value
    _portOutputState |= mask;
    uint8_t inBuffer;
    uint8_t status = I2CManager.read(_I2CAddress, &inBuffer, 1, &_portOutputState, 1);
    if (status != I2C_STATUS_OK)
      inBuffer = 0xff;  // Return ones if can't read
    _portInputState = inBuffer;
  }
  if (_portInputState & mask) 
    result = 1;
  else
    result = 0;
  #ifdef DIAG_IO
  //DIAG(F("PCF8574::_read I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)pin, result);
  #endif
  return result;
}

// Loop function to do background scanning of the input port.
void PCF8574::_loop(unsigned long currentMicros) {
  if (requestBlock.isBusy()) return;  // Do nothing if a port read is in progress
  if (scanActive) {
    // Scan in progress and last request completed, so retrieve port status from buffer
    if (requestBlock.status == I2C_STATUS_OK) 
      _portInputState = inputBuffer[0];
    else
      _portInputState = 0xff;
    scanActive = false;
  } else if (currentMicros - _lastLoopEntry > _portTickTime) {
    scanActive = true;
    // Initiate read of module input register
    I2CManager.queueRequest(&requestBlock);
    _lastLoopEntry = currentMicros;
  }
}

void PCF8574::_display() {
  DIAG(F("PCF8574 VPins:%d-%d I2C:x%x"), (int)_firstVpin, 
    (int)_firstVpin+_nPins-1, (int)_I2CAddress);
}

