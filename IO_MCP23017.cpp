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

IODevice *MCP23017::createInstance(VPIN firstVpin, int nPins, uint8_t I2CAddress, int interruptPin) {
  MCP23017 *dev = new MCP23017();
  dev->_firstVpin = firstVpin;
  dev->_nPins = min(nPins, 16*8);
  uint8_t nModules = (nPins+15)/16;
  dev->_nModules  = nModules;
  dev->_I2CAddress = I2CAddress;
  dev->_gpioInterruptPin = interruptPin;
  // Allocate memory for module state
  uint8_t *blockStart = (uint8_t *)calloc(6, nModules);
  dev->_currentPortState = (uint16_t *)blockStart;  // two bytes per module
  dev->_portMode = (uint16_t *)(blockStart + 2*nModules); // two bytes per module
  dev->_portPullup = (uint16_t *)(blockStart + 4*nModules); // two bytes per module
  addDevice(dev);
  return dev;
}

void MCP23017::create(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin) {
  createInstance(vpin, nPins, I2CAddress, interruptPin);
}
  
// Device-specific initialisation
void MCP23017::_begin() { 
  // Configure pin used for GPIO extender notification of change.
  if (_gpioInterruptPin >= 0)
    pinMode(_gpioInterruptPin, INPUT_PULLUP);

  // Initialise structure for reading GPIO registers
  outputBuffer[0] = REG_GPIOA;
  requestBlock.setRequestParams(0, inputBuffer, sizeof(inputBuffer), outputBuffer, sizeof(outputBuffer));

  I2CManager.begin();
  I2CManager.setClock(1000000);
  for (int i=0; i<_nModules; i++) {
    uint8_t address = _I2CAddress+i;
    if (I2CManager.exists(address))
      DIAG(F("MCP23017 configured on I2C:x%x"), (int)address);
    _portMode[i] = 0xFFFF; // Default to input mode
    _portPullup[i] = 0xFFFF; // Default to pullup enabled
    _currentPortState[i] = 0x0000;
    // Initialise device registers (in case it's warm-starting)
    // IOCON is set MIRROR=1, ODR=1 (open drain shared interrupt pin)
    writeRegister(address, REG_IOCON, 0x44);
    writeRegister2(address, REG_GPIOA, _currentPortState[i] & 0xff, _currentPortState[i]>>8);
    writeRegister2(address, REG_IODIRA, _portMode[i] & 0xff, _portMode[i]>>8);
    writeRegister2(address, REG_GPPUA, _portPullup[i] & 0xff, _portPullup[i]>>8);
    writeRegister2(address, REG_GPINTENA, 0xff, 0xff);  // Enable interrupt mode
  }
}

// Device-specific pin configuration
bool MCP23017::_configurePullup(VPIN vpin, bool pullup) {
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("MCP23017::_configurePullup Pin:%d Val:%d"), vpin, pullup);
  #endif
  int deviceIndex = pin/16;
  pin %= 16; // Pin within device
  uint8_t address = _I2CAddress+deviceIndex;
  uint16_t mask = 1 << pin;
  if (pullup)
    _portPullup[deviceIndex] |= mask;
  else
    _portPullup[deviceIndex] &= ~mask;
  // Only update the register that has changed.
  if (pin < 8)
    I2CManager.write(address, 2, REG_GPPUA, _portPullup[deviceIndex] & 0xff);  
  else
    I2CManager.write(address, 2, REG_GPPUB, _portPullup[deviceIndex] >> 8);  
  // Then read current port state (synchronous call)
  _currentPortState[deviceIndex] = readRegister2(address, REG_GPIOA);
  return true;
}
  
// Device-specific write function.
void MCP23017::_write(VPIN vpin, int value) {
  int pin = vpin-_firstVpin;
  int deviceIndex = pin / 16;
  uint8_t address = _I2CAddress+deviceIndex;
  pin %= 16; // Pin number within device
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)pin, value);
  #endif
  uint16_t mask = 1 << pin;

  if (value) 
    _currentPortState[deviceIndex] |= mask;
  else
    _currentPortState[deviceIndex] &= ~mask;

  // Write updated value to the appropriate port
  if (pin < 8)
    writeRegister(address, REG_GPIOA, _currentPortState[deviceIndex] & 0xff);
  else
    writeRegister(address, REG_GPIOB, _currentPortState[deviceIndex] >> 8);

  // Set port mode to output if not already set
  if (_portMode[deviceIndex] & mask) {
    _portMode[deviceIndex] &= ~mask;
    if (pin < 8)
      writeRegister(address, REG_IODIRA, _portMode[deviceIndex] & 0xff);
    else
      writeRegister(address, REG_IODIRB, _portMode[deviceIndex] >> 8);
  }
}

// Device-specific read function.  If pin previously in write mode, then set read mode and read
//  the port value synchronously.  Subsequent the port reads are done from the _loop function.
int MCP23017::_read(VPIN vpin) {
  int result;
  int pin = vpin-_firstVpin;
  int deviceIndex = pin / 16;
  uint8_t address = _I2CAddress+deviceIndex;
  pin %= 16;
  uint16_t mask = 1 << pin;
  // Set port mode input
  if (!(_portMode[deviceIndex] & mask)) {
    _portMode[deviceIndex] |= mask;
    if (pin < 8)
      writeRegister(address, REG_IODIRA, _portMode[deviceIndex] & 0xff);
    else
      writeRegister(address, REG_IODIRB, _portMode[deviceIndex] >> 8);
    // Then read current port state (synchronous call)
    _currentPortState[deviceIndex] = readRegister2(address, REG_GPIOA);
  }
  if (_currentPortState[deviceIndex] & mask) 
    result = 1;
  else
    result = 0;
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Read I2C:x%x Pin:%d Value:%d"), (int)address, (int)pin, result);
  #endif
  return result;
}

// Loop function to read and refresh port states
void MCP23017::_loop(unsigned long currentMicros) {
  if (requestBlock.isBusy()) return;  // Do nothing if a port read is in progress
  if (currentPollDevice >= 0) {
    // Scan in progress and last request completed, so retrieve port status from buffer
    if (requestBlock.status == I2C_STATUS_OK) 
      _currentPortState[currentPollDevice] = ((uint16_t)inputBuffer[1] << 8) | inputBuffer[0];
    else
      _currentPortState[currentPollDevice] = 0xffff;
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

  if (currentPollDevice >= 0) {
    // Initiate read of next module
    requestBlock.i2cAddress = _I2CAddress+currentPollDevice;
    I2CManager.queueRequest(&requestBlock);
  }
}

// Helper function to write a register
void MCP23017::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  I2CManager.write(address, 2, reg, value);
}

// Helper function to write a register pair (e.g. GPIOA/B, IODIRA/B)
void MCP23017::writeRegister2(uint8_t address, uint8_t reg, uint8_t valueA, uint8_t valueB) {
  I2CManager.write(address, 3, reg, valueA, valueB);
}

// Helper function to read a register pair (e.g. GPIOA/B, IODIRA/B). Returns all ones if error.
uint16_t MCP23017::readRegister2(uint8_t address, uint8_t reg) {
  uint8_t buffer[2];
  uint8_t status = I2CManager.read(address, buffer, 2, &reg, 1);
  if (status == I2C_STATUS_OK) 
    return ((uint16_t)buffer[1] << 8) | buffer[0];
  else  
    return 0xffff;
}

// Display details of this device.
void MCP23017::_display() {
  for (int i=0; i<_nModules; i++) {
    DIAG(F("MCP23017 Vpins:%d-%d I2C:x%x"), (int)_firstVpin+i*16, 
      (int)min(_firstVpin+i*16+15,_firstVpin+_nPins-1), (int)(_I2CAddress+i));
  }
}
