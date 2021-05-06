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

//==================================================================================================================
// Static methods
//------------------------------------------------------------------------------------------------------------------

// Static functions

// Static method to initialise the IODevice subsystem.  
// Create any standard device instances that may be required, such as the Arduino pins 
// and PCA9685.
void IODevice::begin() {
  // Initialise the IO subsystem
  ArduinoPins::create(2, 48);  // Reserve pins numbered 2-49 for direct access
#if !defined(ARDUINO_AVR_NANO) && !defined(ARDUINO_AVR_UNO)
  // Predefine two PCA9685 modules 0x40-0x41
  // Allocates 32 pins 100-131
  PCA9685::create(IODevice::firstServoVpin, 32, 0x40);
  // Predefine one PCF8574 module 0x23
  // Allocates 8 pins 132-139
  PCF8574::create(IODevice::firstServoVpin+32, 8, 0x23);
  // Predefine one MCP23017 module 0x20
  // Allocates 16 pins 164-x179
  MCP23017::create(IODevice::firstServoVpin+64, 16, 0x20);
#endif
}

// Overarching static loop() method for the IODevice subsystem.  Works through the
// list of installed devices and calls their individual _loop() method.
// Devices may or may not implement this, but if they do it is useful for things like animations 
// or flashing LEDs.
// The current value of micros() is passed as a parameter, so the called loop function
// doesn't need to invoke it.
void IODevice::loop() {
  unsigned long currentMicros = micros();
  // Call every device's loop function in turn.
  if (!_nextLoopDevice) _nextLoopDevice = _firstDevice;
  _nextLoopDevice->_loop(currentMicros);
  _nextLoopDevice = _nextLoopDevice->_nextDevice;
  I2CManager.loop();

  // Report loop time if diags enabled
//#if defined(DIAG_IO)
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
//#endif
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
bool IODevice::configure(VPIN vpin, int paramCount, int params[]) {
  IODevice *dev = findDevice(vpin);
  if (dev) return dev->_configure(vpin, paramCount, params);
  return false;
}

// Find device associaed with nominated Vpin and pass request on to it.
//  configurePullup is used invoke a GPIO IODevice instance's _configurePullup method.
//  Return false if not found.
bool IODevice::configurePullup(VPIN vpin, bool pullup) {
  IODevice *dev = findDevice(vpin);
  if (dev) return dev->_configurePullup(vpin, pullup);
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
  DIAG(F("IODevice::write(): Vpin ID %d not found!"), (int)vpin);
#endif
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
  DIAG(F("IODevice::write(): Vpin ID %d not found!"), (int)vpin);
#endif  
} 

// Read value from virtual pin.
bool IODevice::read(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_read(vpin);
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::read(): Vpin %d not found!"), (int)vpin);
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

/////////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor
ArduinoPins::ArduinoPins(VPIN firstVpin, int nPins) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _pinPullups = (uint8_t *)calloc(1, (_nPins+7)/8);
  for (int i=0; i<(_nPins+7)/8; i++) _pinPullups[i] = 0;
}

// Device-specific pin configuration
bool ArduinoPins::_configurePullup(VPIN id, bool pullup) {
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

