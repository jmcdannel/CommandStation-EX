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

// TODO: Enable optional use of interrupt pin on MCP23008/23017 modules, 
// chained from one module to another, into an Arduino digital input.
// When the Arduino digital input is pulled DOWN, it indicates that the 
// modules need to be scanned.  Once the interrupting module(s) has been
// scanned, then the digital input will deactivate (pulled UP).

#include <Arduino.h>
#include "IODevice.h"
#include "DIAG.h" 
#include "FSH.h"
#include "IO_MCP23017.h"

//==================================================================================================================
// Static methods
//------------------------------------------------------------------------------------------------------------------

// Static functions

// Static method to initialise the IODevice subsystem.  

#if !defined(IO_NO_HAL)

// Create any standard device instances that may be required, such as the Arduino pins 
// and PCA9685.
void IODevice::begin() {
  // Initialise the IO subsystem
  ArduinoPins::create(2, 48);  // Reserve pins numbered 2-49 for direct access
  // Predefine two PCA9685 modules 0x40-0x41
  // Allocates 32 pins 100-131
  PCA9685::create(100, 16, 0x40);
  PCA9685::create(116, 16, 0x41);
  // Predefine two MCP23017 module 0x20/0x21
  // Allocates 32 pins 164-195
  MCP23017::create(164, 16, 0x20);
  MCP23017::create(180, 16, 0x21);
}

// Overarching static loop() method for the IODevice subsystem.  Works through the
// list of installed devices and calls their individual _loop() method.
// Devices may or may not implement this, but if they do it is useful for things like animations 
// or flashing LEDs.
// The current value of micros() is passed as a parameter, so the called loop function
// doesn't need to invoke it.
void IODevice::loop() {
  unsigned long currentMicros = micros();
  // Call every device's loop function in turn, one per entry.
  if (!_nextLoopDevice) _nextLoopDevice = _firstDevice;
  _nextLoopDevice->_loop(currentMicros);
  _nextLoopDevice = _nextLoopDevice->_nextDevice;
  
  // Report loop time if diags enabled
#if defined(DIAG_LOOPTIMES)
  static unsigned long lastMicros = 0;
  static unsigned long maxElapsed = 0;
  static unsigned long lastOutputTime = 0;
  static unsigned long count = 0;
  unsigned long elapsed = currentMicros - lastMicros;
  // Ignore long loop counts while message is still outputting
  if (currentMicros - lastOutputTime > 3000UL) {
    if (elapsed > maxElapsed) maxElapsed = elapsed;
  }
  count++;
  if (currentMicros - lastOutputTime > 5000000UL) {
    if (lastOutputTime > 0) 
      LCD(1,F("Loop=%lus,%lus max"), (unsigned long)5000000UL/count, maxElapsed);
    maxElapsed = 0;
    count = 0;
    lastOutputTime = currentMicros;
  }
  lastMicros = micros();
#endif
}

// Display a list of all the devices on the diagnostic stream.
void IODevice::DumpAll() {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    dev->_display();
  }
}

// Determine if the specified vpin is allocated to a device.
bool IODevice::exists(VPIN vpin) {
  return findDevice(vpin) != NULL;
}

// check whether the pin supports notification.  If so, then regular _read calls are not required.
bool IODevice::hasCallback(VPIN vpin) {
  IODevice *dev = findDevice(vpin);
  if (!dev) return false;
  return dev->_hasCallback(vpin);
}


// Remove specified device if one exists.  This is necessary if devices are
// created on-the-fly by Turnouts, Sensors or Outputs since they may have
// been saved to EEPROM and recreated on start.
void IODevice::remove(VPIN vpin) {
  // Only works if the object is exclusive, i.e. only one VPIN.
  IODevice *previousDev = 0;
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) {
      // Found object
      if (dev->_isDeletable()) {
        // First check it isn't next one to be processed by loop().
        //   If so, skip to the following one.
        if (dev == _nextLoopDevice) 
          _nextLoopDevice = _nextLoopDevice->_nextDevice;
        // Now unlink
        if (!previousDev)
          _firstDevice = dev->_nextDevice;
        else
          previousDev->_nextDevice = dev->_nextDevice;
        delete dev;
#ifdef DIAG_IO
        DIAG(F("IODevice deleted Vpin:%d"), vpin);
#endif
        return;
      }
    }
    previousDev = dev;
  }
}

// Display (to diagnostics) details of the device.
void IODevice::_display() {
  DIAG(F("Unknown device Vpins:%d-%d"), (int)_firstVpin, (int)_firstVpin+_nPins-1);
}

// Find device associated with nominated Vpin and pass configuration values on to it.
//   Return false if not found.
bool IODevice::configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  IODevice *dev = findDevice(vpin);
  if (dev) return dev->_configure(vpin, configType, paramCount, params);
  return false;
}

// Write value to virtual pin(s).  If multiple devices are allocated the same pin
//  then only the first one found will be used.
void IODevice::write(VPIN vpin, int value) {
  IODevice *dev = findDevice(vpin);
  if (dev) {
    dev->_write(vpin, value);
    return;
  }
#ifdef DIAG_IO
  //DIAG(F("IODevice::write(): Vpin ID %d not found!"), (int)vpin);
#endif
}

void IODevice::setGPIOInterruptPin(int16_t pinNumber) {
  if (pinNumber >= 0)
    pinMode(pinNumber, INPUT_PULLUP);
  _gpioInterruptPin = pinNumber;
}

IONotifyStateChangeCallback *IODevice::registerInputChangeNotification(IONotifyStateChangeCallback *callback) {
  IONotifyStateChangeCallback *previousHead = _notifyCallbackChain;
  _notifyCallbackChain = callback;
  return previousHead;
}


// Private helper function to add a device to the chain of devices.
void IODevice::addDevice(IODevice *newDevice) {
  // Link new object to the start of chain.  Thereby,
  // a write or read will act on the first device found.
  newDevice->_nextDevice = _firstDevice;
  _firstDevice = newDevice;

  // Initialise device
  newDevice->_begin();
}

// Private helper function to locate a device by VPIN.  Returns NULL if not found
IODevice *IODevice::findDevice(VPIN vpin) { 
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev;
  }
  return NULL;
}
  
//==================================================================================================================
// Static data
//------------------------------------------------------------------------------------------------------------------

IONotifyStateChangeCallback *IODevice::_notifyCallbackChain = 0;


//==================================================================================================================
// Instance members
//------------------------------------------------------------------------------------------------------------------

// Method to check whether the id corresponds to this device
bool IODevice::owns(VPIN id) {
  return (id >= _firstVpin && id < _firstVpin + _nPins);
}

// Write to devices which are after the current one in the list; this 
// function allows a device to have the same input and output VPIN number, and
// a write to the VPIN from outside the device is passed to the device, but a 
// call to writeDownstream will pass it to another device with the same
// VPIN number if one exists.
void IODevice::writeDownstream(VPIN vpin, int value) {
  for (IODevice *dev = _nextDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) {
      dev->_write(vpin, value);
      return;
    }
  }
#ifdef DIAG_IO
  //DIAG(F("IODevice::write(): Vpin ID %d not found!"), (int)vpin);
#endif  
} 

// Read value from virtual pin.
bool IODevice::read(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_read(vpin);
  }
#ifdef DIAG_IO
  //DIAG(F("IODevice::read(): Vpin %d not found!"), (int)vpin);
#endif
  return false;
}

bool IODevice::_isDeletable() {
  return false;
}

// Start of chain of devices.
IODevice *IODevice::_firstDevice = 0;

// Reference to next device to be called on _loop() method.
IODevice *IODevice::_nextLoopDevice = 0;

#else // !defined(IO_NO_HAL)

// Minimal implementations of public HAL interface, to support Arduino pin I/O and nothing more.

void IODevice::begin() { DIAG(F("NO HAL CONFIGURED!")); }
bool IODevice::configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  (void)vpin; (void)paramCount; (void)params; // Avoid compiler warnings
  if (configType == CONFIGURE_INPUT || configType == CONFIGURE_OUTPUT) 
    return true;
  else
    return false;
}
void IODevice::write(VPIN vpin, int value) {
  pinMode(vpin, OUTPUT);
  digitalWrite(vpin, value);
}
bool IODevice::hasCallback(VPIN vpin) { 
  (void)vpin;  // Avoid compiler warnings
  return false; 
}
bool IODevice::read(VPIN vpin) { 
  pinMode(vpin, INPUT_PULLUP);
  return digitalRead(vpin);
}
void IODevice::loop() {}
void IODevice::DumpAll() {
  DIAG(F("NO HAL CONFIGURED!"));
}
bool IODevice::exists(VPIN vpin) { return (vpin > 2 && vpin < 49); }
void IODevice::remove(VPIN vpin) {
  (void)vpin;  // Avoid compiler warnings
}
void IODevice::setGPIOInterruptPin(int16_t pinNumber) {
  (void) pinNumber; // Avoid compiler warning
}
IONotifyStateChangeCallback *IODevice::registerInputChangeNotification(IONotifyStateChangeCallback *callback) {
  (void)callback;  // Avoid compiler warning
  return NULL;
}

#endif // IO_NO_HAL


/////////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor
ArduinoPins::ArduinoPins(VPIN firstVpin, int nPins) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _pinPullups = (uint8_t *)calloc(1, (_nPins+7)/8);
  for (int i=0; i<(_nPins+7)/8; i++) _pinPullups[i] = 0;
}

// Device-specific pin configuration
bool ArduinoPins::_configure(VPIN id, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_INPUT) return false;
  if (paramCount != 1) return false;
  bool pullup = params[0];

  int pin = id;
  #ifdef DIAG_IO
  DIAG(F("Arduino _configurePullup Pin:%d Val:%d"), pin, pullup);
  #endif
  uint8_t mask = 1 << ((pin-_firstVpin) % 8);
  uint8_t index = (pin-_firstVpin) / 8;
  if (pullup) {
    _pinPullups[index] |= mask;
    pinMode(pin, INPUT_PULLUP);
  } else {
    _pinPullups[index] &= ~mask;
    pinMode(pin, INPUT);
  }
  return true;
}

// Device-specific write function.
void ArduinoPins::_write(VPIN id, int value) {
  int pin = id;
  #ifdef DIAG_IO
  DIAG(F("Arduino Write Pin:%d Val:%d"), pin, value);
  #endif
  digitalWrite(pin, value);
  pinMode(pin, OUTPUT);
}

// Device-specific read function.
int ArduinoPins::_read(VPIN id) {
  int pin = id;
  uint8_t mask = 1 << ((pin-_firstVpin) % 8);
  uint8_t index = (pin-_firstVpin) / 8;
  if (_pinPullups[index] & mask) 
    pinMode(pin, INPUT_PULLUP);
  else
    pinMode(pin, INPUT);
  int value = digitalRead(pin);
  #ifdef DIAG_IO
  //DIAG(F("Arduino Read Pin:%d Value:%d"), pin, value);
  #endif
  return value;
}

void ArduinoPins::_display() {
  DIAG(F("Arduino Vpins:%d-%d"), (int)_firstVpin, (int)_firstVpin+_nPins-1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////