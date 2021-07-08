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

// TODO: Consider making GPIOBase a template class to reduce RAM usage for 8-pin modules.
// I have tried this but couldn't get it to compile.

#include "IO_GPIOBase.h"

// Constructor
GPIOBase::GPIOBase(FSH *deviceName, VPIN firstVpin, uint8_t nPins, uint8_t I2CAddress, int interruptPin) {
  _deviceName = deviceName;
  _firstVpin = firstVpin;
  _nPins = nPins;
  _I2CAddress = I2CAddress;
  _gpioInterruptPin = interruptPin;
  _notifyCallbackChain = 0;
  // Add device to list of devices.
  addDevice(this);

  // Configure pin used for GPIO extender notification of change (if allocated)
  if (_gpioInterruptPin >= 0) 
    pinMode(_gpioInterruptPin, INPUT_PULLUP);

  I2CManager.begin();
  I2CManager.setClock(400000);
  if (I2CManager.exists(I2CAddress)) {
    _display();
    _portMode = 0x0000;  // default to input mode
    _portPullup = 0xFFFF; // default to pullup enabled
    _portInputState = 0x0000; 
  }
  _deviceState = DEVSTATE_NORMAL;
  _lastLoopEntry = micros();
}

void GPIOBase::_begin() {}

bool GPIOBase::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount != 1) return false;
  bool pullup = params[0];
  int pin = vpin - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("%S I2C:x%x Config Pin:%d Val:%d"), _deviceName, _I2CAddress, pin, pullup);
  #endif
  uint16_t mask = 1 << pin;
  if (pullup) 
    _portPullup |= mask;
  else
    _portPullup &= ~mask;

  // Call subclass's virtual function to write to device
  _writePullups();
  // Re-read port following change
  _readGpioPort();

  return true;
}

// Periodically read the input port
void GPIOBase::_loop(unsigned long currentMicros) {
  #ifdef DIAG_IO
  uint16_t lastPortStates = _portInputState;
  #endif
  if (_deviceState == DEVSTATE_SCANNING && !requestBlock.isBusy()) {
    uint8_t status = requestBlock.status;
    if (status == I2C_STATUS_OK) {
      _deviceState = DEVSTATE_NORMAL;
    } else {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("%S I2C:x%x Error:%d"), _deviceName, _I2CAddress, status);
    }
    _processCompletion(status);
  }
  // Check if interrupt configured.  If so, and pin is not pulled down, finish.
  if (_gpioInterruptPin >= 0) {
    if (digitalRead(_gpioInterruptPin)) return;
  } else
  // No interrupt pin.  Check if tick has elapsed.  If not, finish.
  if (currentMicros - _lastLoopEntry < _portTickTime) return;

  // TODO: Could suppress reads if there are no pins configured as inputs!

  // Read input
  _lastLoopEntry = currentMicros;
  if (_deviceState == DEVSTATE_NORMAL) {
    _readGpioPort(false);  // Initiate non-blocking read
    _deviceState= DEVSTATE_SCANNING;
  }

  #ifdef DIAG_IO
  uint16_t differences = lastPortStates ^ _portInputState;
  if (differences)
    DIAG(F("%S I2C:x%x PortStates:%x"), _deviceName, _I2CAddress, _portInputState);
  #endif
}

void GPIOBase::_display() {
  DIAG(F("%S I2C:x%x Configured on Vpins:%d-%d"), _deviceName, _I2CAddress, 
    _firstVpin, _firstVpin+_nPins-1);
}

void GPIOBase::_write(VPIN vpin, int value) {
  int pin = vpin - _firstVpin;
  uint16_t mask = 1 << pin;
  #ifdef DIAG_IO
  DIAG(F("%S I2C:x%x Write Pin:%d Val:%d"), _deviceName, _I2CAddress, pin, value);
  #endif

  // Set port mode output
  if (!(_portMode & mask)) {
    _portMode |= mask;
    _writePortModes();
  }

  // Update port output state
  if (value) 
    _portOutputState |= mask;
  else
    _portOutputState &= ~mask;

  // Call subclass's virtual function to write to device.
  return _writeGpioPort();
}

int GPIOBase::_read(VPIN vpin) {
  int pin = vpin - _firstVpin;
  uint16_t mask = 1 << pin;

  // Set port mode to input
  if (_portMode & mask) {
    _portMode &= ~mask;
    _writePortModes();
    // Port won't have been read yet, so read it now.
    _readGpioPort();
    #ifdef DIAG_IO
    DIAG(F("%S I2C:x%x PortStates:%x"), _deviceName, _I2CAddress, _portInputState);
    #endif
  }
  return (_portInputState & mask) ? 1 : 0;
}
