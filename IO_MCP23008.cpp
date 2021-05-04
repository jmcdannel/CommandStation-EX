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

// Define symbol to enable MCP23008 input port value caching (reduce I2C traffic).
#define MCP23008_OPTIMISE

// Constructor
MCP23008::MCP23008() {}

IODevice *MCP23008::createInstance(VPIN vpin, int nPins, uint8_t I2CAddress) {
  #ifdef DIAG_IO
  DIAG(F("MCP23008 created Vpins:%d-%d I2C:%x"), vpin, vpin+nPins-1, I2CAddress);
  #endif
  MCP23008 *dev = new MCP23008();
  dev->_firstVpin = vpin;
  dev->_nPins = min(nPins, 8*8);
  uint8_t nModules = (dev->_nPins + 7) / 8; // Number of modules in use.
  dev->_nModules = nModules;
  dev->_I2CAddress = I2CAddress;
  // Allocate memory for module state
  uint8_t *blockStart = (uint8_t *)calloc(4, nModules);
  dev->_portDirection = blockStart;
  dev->_portPullup = blockStart + nModules;
  dev->_portInputState = blockStart + 2*nModules;
  dev->_portOutputState = blockStart + 3*nModules;
  addDevice(dev);
  return dev;
}

// Parameters: firstVPIN, nPins
// We allow up to 8 devices, on successive I2C addresses starting 
// with the specified one.  VPINS are allocated contiguously, 8 
// per device.
void MCP23008::create(VPIN vpin, int nPins, uint8_t I2CAddress) {
  createInstance(vpin, nPins, I2CAddress);
}

// Device-specific initialisation
void MCP23008::_begin() {
  // Initialise structure for reading GPIO ports
  requestBlock.setRequestParams(0, inputBuffer, sizeof(inputBuffer), outputBuffer, sizeof(outputBuffer));
  inputBuffer[0] = REG_GPIO;

  I2CManager.begin();
  I2CManager.setClock(1000000);  // Supports fast clock
  for (int i=0; i<_nModules; i++) {
    if (I2CManager.exists(_I2CAddress+i))
      DIAG(F("MCP23008 configured on I2C:x%x"), (int)_I2CAddress+i);
    _portDirection[i] = 0xff; // Defaults to Input mode
    _portPullup[i] = 0x00; // Defaults to no pullup
    _portInputState[i] = 0x00; 
    _portOutputState[i] = 0x00; // Defaults to output zero.
    // Initialise device (in case it's warm-starting)
    I2CManager.write(_I2CAddress+i, 2, REG_GPIO, _portOutputState[i]);
    I2CManager.write(_I2CAddress+i, 2, REG_IODIR, _portDirection[i]);
    I2CManager.write(_I2CAddress+i, 2, REG_GPPU, _portPullup[i]);
  }
}

// Device-specific pin configuration
bool MCP23008::_configurePullup(VPIN vpin, bool pullup) {
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("MCP23008 _configurePullup Pin:%d Val:%d"), vpin, pullup);
  #endif
  int deviceIndex = pin/8;
  pin %= 8; // Pin within device
  uint8_t mask = 1 << pin;
  if (pullup) 
    _portPullup[deviceIndex] |= mask;
  else
    _portPullup[deviceIndex] &= ~mask;
  I2CManager.write(_I2CAddress+deviceIndex, 2, REG_GPPU, _portPullup[deviceIndex]);  
  return true;
}


// Device-specific write function.
void MCP23008::_write(VPIN vpin, int value) {
  int pin = vpin -_firstVpin;
  int deviceIndex = pin / 8;
  pin %= 8; // Pin within device
  #ifdef DIAG_IO
  DIAG(F("MCP23008 Write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)vpin, value);
  #endif
  uint8_t mask = 1 << pin;
  if (value) 
    _portOutputState[deviceIndex] |= mask;
  else
    _portOutputState[deviceIndex] &= ~mask;
  writeRegister(_I2CAddress+deviceIndex, REG_GPIO, _portOutputState[deviceIndex]);
  // Check port mode
  if (_portDirection[deviceIndex] & mask) {
    // Currently in read mode, so set to write mode
    _portDirection[deviceIndex] &= ~mask;
    writeRegister(_I2CAddress+deviceIndex, REG_IODIR, _portDirection[deviceIndex]);
  }
  // Assume that writing to the port invalidates any cached read, so set the port counter to 0
  //  to force the port to be refreshed next time a read is issued.
  _portCounter[deviceIndex] = 0;
}

// Device-specific read function.  If port was in write mode, then set to read mode and 
// read the value of the port immediately.  Subsequent reads are done asynchronously
// within the loop() function.
int MCP23008::_read(VPIN vpin) {
  int result;
  int pin = vpin-_firstVpin;
  int deviceIndex = pin / 8;  
  pin %= 8;
  uint8_t mask = 1 << pin;
  if (!(_portDirection[deviceIndex] & mask)) {
    // Pin currently in write mode, so set to read mode
    _portDirection[deviceIndex] |= mask;
    writeRegister(_I2CAddress+deviceIndex, REG_IODIR, _portDirection[deviceIndex]);
    // Read GPIO register
    _portInputState[deviceIndex] = readRegister(_I2CAddress+deviceIndex, REG_GPIO);
  }
  if (_portInputState[deviceIndex] & mask) 
    result = 1;
  else
    result = 0;
  #ifdef DIAG_IO
  //DIAG(F("MCP23008 Read I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)pin, result);
  #endif
  return result;
}

// Loop function to maintain timers associated with port read optimisation.  Decrement port counters
// periodically.  When the portCounter reaches zero, the port value is considered to be out-of-date
// and will need re-reading.
void MCP23008::_loop(unsigned long currentMicros) {
  if (requestBlock.isBusy()) return;  // Do nothing if a port read is in progress
  if (currentPollDevice >= 0) {
    // Scan in progress and last request completed, so retrieve port status from buffer
    if (requestBlock.status == I2C_STATUS_OK) 
      _portInputState[currentPollDevice] = inputBuffer[0];
    else
      _portInputState[currentPollDevice] = 0xff;
    if (++currentPollDevice >= _nModules)
      currentPollDevice = -1;  // Scan completed
  } else if (currentMicros - _lastLoopEntry > _portTickTime) {
    if (_gpioInterruptPin < 0 || digitalRead(_gpioInterruptPin) == 0) {
      // A cyclic scan is due, and either a GPIO device has pulled down its interrupt pin to
      //  signal something has changed or GPIO device interrupts are not configured
      currentPollDevice = 0;  // Starting new scan.
    }
    _lastLoopEntry = currentMicros;
  }

  // Initiate read of next module
  requestBlock.i2cAddress = _I2CAddress+currentPollDevice;
  I2CManager.queueRequest(&requestBlock);
}

void MCP23008::_display() {
  for (int i=0; i<_nModules; i++) {
    DIAG(F("MCP23008 Vpins:%d-%d I2C:x%x"), (int)_firstVpin+i*8, 
      (int)min(_firstVpin+i*8+7,_firstVpin+_nPins-1), (int)(_I2CAddress+i));
  }
}

// Helper function to write a register
void MCP23008::writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t value) {
  I2CManager.write(I2CAddress, 2, reg, value);
}

// Helper function to read a register.  Returns zero if error.
uint8_t MCP23008::readRegister(uint8_t I2CAddress, uint8_t reg) {
  uint8_t buffer;
  uint8_t status = I2CManager.read(I2CAddress, &buffer, 1, &reg, 1);
  if (status == I2C_STATUS_OK) 
    return buffer;
  else
    return 0;
}

