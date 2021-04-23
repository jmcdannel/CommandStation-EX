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

// Constructor
MCP23017::MCP23017() {}

IODevice *MCP23017::createInstance(VPIN firstID, int nPins, uint8_t I2CAddress) {
  MCP23017 *dev = new MCP23017();
  dev->_firstID = firstID;
  dev->_nPins = min(nPins, 16*8);
  dev->_nModules = (nPins+15)/16;
  dev->_I2CAddress = I2CAddress;
  addDevice(dev);
  dev->_begin();
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
    if (I2CManager.exists(_I2CAddress+i))
      DIAG(F("MCP23017 on I2C:x%x"), _I2CAddress+i);
    _portModeA[i] = 0xff; // Default to read mode
    _portModeB[i] = 0xff;
    _currentPortStateA[i] = 0;
    _currentPortStateB[i] = 0;
  }
}
  
// Device-specific write function.
// TODO: Cache port mode so that it doesn't have to
// be changed every time.
void MCP23017::_write(VPIN vpin, int value) {
  int pin = vpin-_firstID;
  int deviceIndex = pin / 16;
  pin %= 16; // Pin number within device
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)pin, value);
  #endif
  uint8_t mask = 1 << pin;

  if (deviceIndex < 8) {
    if (value) 
      _currentPortStateA[deviceIndex] |= mask;
    else
      _currentPortStateA[deviceIndex] &= ~mask;
    // Write values
    writeRegister(_I2CAddress, GPIOA, _currentPortStateA[deviceIndex]);
    // Set port mode output
    _portModeA[deviceIndex] &= ~mask;
    writeRegister(_I2CAddress, IODIRA, _portModeA[deviceIndex]);
  } else {
    if (value) 
      _currentPortStateB[deviceIndex] |= mask;
    else
      _currentPortStateB[deviceIndex] &= ~mask;
    // Write values
    writeRegister(_I2CAddress, GPIOB, _currentPortStateB[deviceIndex]);
    // Set port mode output
    _portModeB[deviceIndex] &= ~mask;
    writeRegister(_I2CAddress, IODIRB, _portModeB[deviceIndex]);
  }
}

// Device-specific read function.
// TODO: Reduce number of port reads by cacheing 
// the port value, so that a call from _read
// can use the cached value if (a) it's not too
// old and (b) the port mode hasn't been changed.
// TODO: Cache port mode so that it doesn't have to
// be changed every time.
int MCP23017::_read(VPIN vpin) {
  int result;
  uint8_t buffer = 0;
  int pin = vpin-_firstID;
  int deviceIndex = pin / 16;
  pin %= 16;
  uint8_t mask = 1 << (pin % 16);
  if (pin < 8) {
    // Set port mode input
    _portModeA[deviceIndex] |= mask;
    writeRegister(_I2CAddress, IODIRA, _portModeA[deviceIndex]);
    // Read GPIO register values
    buffer = readRegister(_I2CAddress, GPIOA);
  } else {
    // Set port mode input
    _portModeB[deviceIndex] |= mask;
    writeRegister(_I2CAddress, IODIRB, _portModeB[deviceIndex]);
    // Read GPIO register values
    buffer = readRegister(_I2CAddress, GPIOB);
  }
  if (buffer & mask) 
    result = 1;
  else
    result = 0;
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Read I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress, (int)pin, result);
  #endif
  return result;
}

// Helper function to write a register
void MCP23017::writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t value) {
  I2CManager.write(I2CAddress, 2, reg, value);
}

// Helper function to read a register
uint8_t MCP23017::readRegister(uint8_t I2CAddress, uint8_t reg) {
  uint8_t buffer;
  I2CManager.read(I2CAddress, &buffer, 1, &reg, 1);
  return buffer;
}

// Display details of this device.
void MCP23017::_display() {
  for (int i=0; i<_nModules; i++) {
    DIAG(F("MCP23017 VPins:%d-%d I2C:x%x"), (int)_firstID+i*16, 
      (int)min(_firstID+i*16+15,_firstID+_nPins-1), (int)(_I2CAddress+i));
  }
}
