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

#define MCP23017_OPTIMISE 

// Constructor
MCP23017::MCP23017() {}

IODevice *MCP23017::createInstance(VPIN firstID, int nPins, uint8_t I2CAddress) {
  MCP23017 *dev = new MCP23017();
  dev->_firstID = firstID;
  dev->_nPins = min(nPins, 16*8);
  uint8_t nModules = (nPins+15)/16;
  dev->_nModules  = nModules;
  dev->_I2CAddress = I2CAddress;
  // Allocate memory for module state
  uint8_t *blockStart = (uint8_t *)calloc(7, nModules);
  dev->_currentPortState = (uint16_t *)blockStart;  // two bytes per module
  dev->_portModeA = blockStart + 2*nModules; // one byte per module
  dev->_portModeB = blockStart + 3*nModules; // one byte per module
  dev->_portPullupA = blockStart + 4*nModules; // one byte per module
  dev->_portPullupB = blockStart + 5*nModules; // one byte per module
  dev->_portCounter = blockStart + 6*nModules; // one byte per module
  addDevice(dev);
  return dev;
}

void MCP23017::create(VPIN vpin, int nPins, uint8_t I2CAddress) {
  createInstance(vpin, nPins, I2CAddress);
}
  
// Device-specific initialisation
void MCP23017::_begin() { 
  I2CManager.begin();
  I2CManager.setClock(1000000);
  for (int i=0; i<_nModules; i++) {
    uint8_t address = _I2CAddress+i;
    if (I2CManager.exists(address))
      DIAG(F("MCP23017 on I2C:x%x"), address);
    _portModeA[i] = 0xFF; // Default to read mode
    _portModeB[i] = 0xFF;
    _portPullupA[i] = 0x00; // Default to pullup disabled
    _portPullupB[i] = 0x00;
    _currentPortState[i] = 0x0000;
    // Initialise device registers (in case it's warm-starting)
    writeRegister(address, IOCON, 0x00);
    writeRegister2(address, GPIOA, _currentPortState[i] & 0xff, _currentPortState[i]>>8);
    writeRegister2(address, IODIRA, _portModeA[i], _portModeB[i]);
    writeRegister2(address, GPPUA, _portPullupA[i], _portPullupB[i]);
  }
}
  
// Device-specific write function.
void MCP23017::_write(VPIN vpin, int value) {
  int pin = vpin-_firstID;
  int deviceIndex = pin / 16;
  uint8_t address = _I2CAddress+deviceIndex;
  pin %= 16; // Pin number within device
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)pin, value);
  #endif
  uint8_t mask = 1 << (pin % 8);

  // Write only to registers that need to change.
  if (pin < 8) {
    if (value) 
      _currentPortState[deviceIndex] |= mask;
    else
      _currentPortState[deviceIndex] &= ~mask;
    // Write values
    writeRegister(address, GPIOA, _currentPortState[deviceIndex] & 0xff);
    // Set port mode output if not already set
    if (_portModeA[deviceIndex] & mask) {
      _portModeA[deviceIndex] &= ~mask;
      writeRegister(address, IODIRA, _portModeA[deviceIndex]);
    }
  } else {
    if (value) 
      _currentPortState[deviceIndex] |= ((uint16_t)mask << 8);
    else
      _currentPortState[deviceIndex] &= ~((uint16_t)mask << 8);
    // Write values
    writeRegister(address, GPIOB, (uint8_t)(_currentPortState[deviceIndex] >> 8));
    // Set port mode output if not already set
    if (_portModeB[deviceIndex] & mask) {
      _portModeB[deviceIndex] &= ~mask;
      writeRegister(address, IODIRB, _portModeB[deviceIndex]);
    }
  }
  // Assume that writing to the port invalidates any cached read, so set the port counter to 0
  //  to force the port to be refreshed next time a read is issued.
  _portCounter[deviceIndex] = 0;
}

// Device-specific read function.
// Reduce number of port reads by caching the port value, so that a call from _read
// can use the cached value if (a) it's not too old and (b) the port mode 
// hasn't been changed.  When writing, only write to ports that have to change;
// but read both GPIOA/B at the same time to optimise reads.
int MCP23017::_read(VPIN vpin) {
  int result;
  int pin = vpin-_firstID;
  int deviceIndex = pin / 16;
  uint8_t address = _I2CAddress+deviceIndex;
  pin %= 16;
  uint16_t mask = 1 << (pin % 8);
  if (pin < 8) {
    // Set port mode input
    if (!(_portModeA[deviceIndex] & mask)) {
      _portModeA[deviceIndex] |= mask;
      writeRegister(address, IODIRA, _portModeA[deviceIndex]);
      _portCounter[deviceIndex] = 0;
    }
    // Set pullup
    if (!(_portPullupA[deviceIndex] & mask)) {
      _portPullupA[deviceIndex] |= mask;
      writeRegister(address, GPPUA, _portPullupA[deviceIndex]);
      _portCounter[deviceIndex] = 0;
    }
  } else {
    if (!(_portModeB[deviceIndex] & mask)) {
      _portModeB[deviceIndex] |= mask;
      writeRegister(address, IODIRB, _portModeB[deviceIndex]);
      _portCounter[deviceIndex] = 0;
    }
    // Set pullup
    if (!(_portPullupB[deviceIndex] & mask)) {
      _portPullupB[deviceIndex] |= mask;
      writeRegister(address, GPPUB, _portPullupB[deviceIndex]);
      _portCounter[deviceIndex] = 0;
    }
  }
  if (_portCounter[deviceIndex] == 0) {
    // Read both GPIO ports
    _currentPortState[deviceIndex] = readRegister2(address, GPIOA);
  #ifdef MCP23017_OPTIMISE
    _portCounter[deviceIndex] = _minTicksBetweenPortReads;
  #endif
  }
  mask = 1 << pin;
  if (_currentPortState[deviceIndex] & mask) 
    result = 1;
  else
    result = 0;
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Read I2C:x%x Pin:%d Value:%d"), (int)address, (int)pin, result);
  #endif
  return result;
}

// Loop function to maintain timers associated with port read optimisation.  Decrement port counters
// periodically.  When the portCounter reaches zero, the port value is considered to be out-of-date
// and will need re-reading.
void MCP23017::_loop(unsigned long currentMicros) {
  (void)currentMicros;  // suppress compiler not-used warning.
#ifdef MCP23017_OPTIMISE
  // Process every tick time
  if (currentMicros - _lastLoopEntry > _portTickTime) {
    for (int deviceIndex=0; deviceIndex < _nModules; deviceIndex++) {
      if (_portCounter[deviceIndex] > 0)
        _portCounter[deviceIndex]--;
    }
    _lastLoopEntry = currentMicros;
  }
#endif
}

// Helper function to write a register
void MCP23017::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  I2CManager.write(address, 2, reg, value);
}

// Helper function to write a register pair (e.g. GPIOA/B, IODIRA/B)
void MCP23017::writeRegister2(uint8_t address, uint8_t reg, uint8_t valueA, uint8_t valueB) {
  I2CManager.write(address, 3, reg, valueA, valueB);
}

// Helper function to read a register pair (e.g. GPIOA/B, IODIRA/B)
uint16_t MCP23017::readRegister2(uint8_t address, uint8_t reg) {
  uint8_t buffer[2];
  I2CManager.read(address, buffer, 2, &reg, 1);
  uint16_t result = ((uint16_t)buffer[1] << 8) | buffer[0];
  return result;
}

// Display details of this device.
void MCP23017::_display() {
  for (int i=0; i<_nModules; i++) {
    DIAG(F("MCP23017 VPins:%d-%d I2C:x%x"), (int)_firstID+i*16, 
      (int)min(_firstID+i*16+15,_firstID+_nPins-1), (int)(_I2CAddress+i));
  }
}
