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

// Pre-declare helper functions
static void writeRegister(uint8_t address, uint8_t reg, uint8_t value);
static void writeRegister2(uint8_t address, uint8_t reg, uint8_t valueA, uint8_t valueB);
static uint16_t readRegister2(uint8_t address, uint8_t reg);

// Constructor
MCP23017::MCP23017(VPIN firstVpin, int nPins, uint8_t I2CAddress, int interruptPin) {
  _firstVpin = firstVpin;
  _nPins = min(nPins, 16);
  _I2CAddress = I2CAddress;
  _gpioInterruptPin = interruptPin;
  _notifyCallbackChain = 0;

  // Configure pin used for GPIO extender notification of change.
  if (_gpioInterruptPin >= 0)
    pinMode(_gpioInterruptPin, INPUT_PULLUP);

  // Initialise structure for reading GPIO registers
  outputBuffer[0] = REG_GPIOA;
  requestBlock.setRequestParams(_I2CAddress, inputBuffer, sizeof(inputBuffer), outputBuffer, sizeof(outputBuffer));

  I2CManager.begin();
  I2CManager.setClock(1000000);
  if (I2CManager.exists(I2CAddress))
    DIAG(F("MCP23017 configured Vpins:%d-%d I2C:%x"), _firstVpin, _firstVpin+nPins-1, _I2CAddress);
  _portMode = 0xFFFF; // Default to input mode
  _portPullup = 0xFFFF; // Default to pullup enabled
  _currentPortState = 0x0000;
  // Initialise device registers (in case it's warm-starting)
  // IOCON is set MIRROR=1, ODR=1 (open drain shared interrupt pin)
  writeRegister(I2CAddress, REG_IOCON, 0x44);
  writeRegister2(I2CAddress, REG_GPIOA, _currentPortState & 0xff, _currentPortState>>8);
  writeRegister2(I2CAddress, REG_IODIRA, _portMode & 0xff, _portMode>>8);
  writeRegister2(I2CAddress, REG_GPPUA, _portPullup & 0xff, _portPullup>>8);
  writeRegister2(I2CAddress, REG_GPINTENA, 0xff, 0xff);  // Enable interrupt mode
}

// Static create() method
void MCP23017::create(VPIN firstVpin, int nPins, uint8_t I2CAddress, int interruptPin) {
  MCP23017 *dev = new MCP23017(firstVpin, nPins, I2CAddress, interruptPin);
  addDevice(dev);
}
  
// Device-specific initialisation
void MCP23017::_begin() { 
}

// Device-specific pin configuration
bool MCP23017::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount != 1) return false;
  bool pullup = params[0];
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("MCP23017::_configurePullup Pin:%d Val:%d"), vpin, pullup);
  #endif
  uint16_t mask = 1 << pin;
  if (pullup)
    _portPullup |= mask;
  else
    _portPullup &= ~mask;
  // Only update the register that has changed.
  if (pin < 8)
    I2CManager.write(_I2CAddress, 2, REG_GPPUA, _portPullup & 0xff);  
  else
    I2CManager.write(_I2CAddress, 2, REG_GPPUB, _portPullup >> 8);  
  // Then read current port state (synchronous call)
  _currentPortState = readRegister2(_I2CAddress, REG_GPIOA);
  return true;
}
  
// Device-specific write function.
void MCP23017::_write(VPIN vpin, int value) {
  #ifdef DIAG_IO
  //DIAG(F("MCP23017 Write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress+deviceIndex, (int)pin, value);
  #endif
  int pin = vpin - _firstVpin;
  uint16_t mask = 1 << pin;

  if (value) 
    _currentPortState |= mask;
  else
    _currentPortState &= ~mask;

  // Write updated value to the appropriate port
  if (pin < 8)
    writeRegister(_I2CAddress, REG_GPIOA, _currentPortState & 0xff);
  else
    writeRegister(_I2CAddress, REG_GPIOB, _currentPortState >> 8);

  // Set port mode to output if not already set
  if (_portMode & mask) {
    _portMode &= ~mask;
    if (pin < 8)
      writeRegister(_I2CAddress, REG_IODIRA, _portMode & 0xff);
    else
      writeRegister(_I2CAddress, REG_IODIRB, _portMode >> 8);
  }
}

// Function called to check whether callback notification is supported by this pin.
bool MCP23017::_hasCallback(VPIN vpin) {
  (void) vpin;  // suppress compiler warning.
  return true;
}

// Device-specific read function.  If pin previously in write mode, then set read mode and read
//  the port value synchronously.  Subsequent the port reads are done from the _loop function.
int MCP23017::_read(VPIN vpin) {
  int result;
  int pin = vpin-_firstVpin;
  uint16_t mask = 1 << pin;
  // Set port mode input
  if (!(_portMode & mask)) {
    _portMode |= mask;
    if (pin < 8)
      writeRegister(_I2CAddress, REG_IODIRA, _portMode & 0xff);
    else
      writeRegister(_I2CAddress, REG_IODIRB, _portMode >> 8);
    // Then read current port state (synchronous call)
    _currentPortState = readRegister2(_I2CAddress, REG_GPIOA);
  }
  if (_currentPortState & mask) 
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
  uint16_t inputStates;
  if (requestBlock.isBusy()) return;  // Do nothing if a port read is in progress
  if (scanActive) {
    // Scan in progress and last request completed, so retrieve port status from buffer
    if (requestBlock.status == I2C_STATUS_OK)  {
      inputStates = ((uint16_t)inputBuffer[1] << 8) | inputBuffer[0];
    } else
      inputStates = 0xffff;
    uint16_t differences = inputStates ^ _currentPortState;
     // Save input states
    _currentPortState = inputStates;
    scanActive = false;

    // Scan for changes in input states and invoke callback
    if (differences && (_notifyCallbackChain != NULL)) {
      // Scan for differences bit by bit
      uint16_t mask = 1;
      for (int pin=0; pin<16; pin++) {
        if (differences & mask) {
          // Change detected.
          _notifyCallbackChain(_firstVpin+pin, (_currentPortState & mask) != 0);
        }
        mask <<= 1;
      }
    }
    
  } else if (currentMicros - _lastLoopEntry > _portTickTime) {
    if (_gpioInterruptPin < 0 || digitalRead(_gpioInterruptPin) == 0) {
      // A cyclic scan is due, and either a GPIO device has pulled down its interrupt pin to
      //  signal something has changed or GPIO device interrupts are not configured
      scanActive = true;  // Starting new scan.
      // Initiate read
      I2CManager.queueRequest(&requestBlock);
    }
    _lastLoopEntry = currentMicros;
  }
}

// Helper function to write a register
static void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  I2CManager.write(address, 2, reg, value);
}

// Helper function to write a register pair (e.g. GPIOA/B, IODIRA/B)
static void writeRegister2(uint8_t address, uint8_t reg, uint8_t valueA, uint8_t valueB) {
  I2CManager.write(address, 3, reg, valueA, valueB);
}

// Helper function to read a register pair (e.g. GPIOA/B, IODIRA/B). Returns all ones if error.
static uint16_t readRegister2(uint8_t address, uint8_t reg) {
  uint8_t buffer[2];
  uint8_t status = I2CManager.read(address, buffer, 2, &reg, 1);
  if (status == I2C_STATUS_OK) 
    return ((uint16_t)buffer[1] << 8) | buffer[0];
  else  
    return 0xffff;
}

// Display details of this device.
void MCP23017::_display() {
  DIAG(F("MCP23017 Vpins:%d-%d I2C:x%x"), (int)_firstVpin, 
    _firstVpin+_nPins-1, (int)_I2CAddress);
}
