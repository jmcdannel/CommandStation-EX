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

#include <Arduino.h>
#include "IODevice.h"
#include "DIAG.h" 
#include "FSH.h"

//==================================================================================================================
// Static methods
//------------------------------------------------------------------------------------------------------------------

#ifdef IO_LATEBINDING
// General static method for creating an arbitrary device without having to know
//  its class at coding time.  Late binding, Microsoft call it.
//  The device class is identifed from the deviceType parameter by looking down 
//  a list of registered types.
IODevice *IODevice::create(int deviceType, VPIN firstID, int paramCount, int params[]) {
  for (IODeviceType *dt = _firstDeviceType; dt != 0; dt=dt->_nextDeviceType){
    if (dt->getDeviceType() == deviceType) {
      IODevice *dev = (dt->createFunction)(firstID);
      if (dev)
        dev->_configure(firstID, paramCount, params);
      return dev;
    }
  }
  return NULL;
}

void IODevice::_registerDeviceType(int deviceTypeID, IODevice *createFunction(VPIN)) {
  IODeviceType *dt = new IODeviceType(deviceTypeID);
  // Link new DeviceType into chain
  dt->_nextDeviceType = _firstDeviceType;
  dt->createFunction = createFunction;
  _firstDeviceType = dt;
}
#endif


// Static functions

// Static method to initialise the IODevice subsystem.  
// Create any standard device instances that may be required, such as the Arduino pins 
// and PCA9685.  I've opted to include one PCA9685 for servo control if we're not on a
// nano or uno controller.
void IODevice::begin() {
  // Initialise the IO subsystem
  ArduinoPins::create(2, 48);  // Reserve pins numbered 2-49 for direct access
#if !defined(ARDUINO_AVR_NANO) && !defined(ARDUINO_AVR_UNO)
  // Predefine two PCA9685 modules 0x40-0x41
  // Allocates 32 pins 100-131
  PCA9685::create(IODevice::firstServoVPin, 32, 0x40);
  // Predefine four PCF8574 modules 0x20-0x23
  // Allocates 32 pins 132-163
  PCF8574::create(IODevice::firstServoVPin+32, 32, 0x20);
  // Predefine two MCP23017 modules 0x24-0x25
  // Allocates 32 pins 164-x195
  MCP23017::create(IODevice::firstServoVPin+64, 32, 0x24);
#endif
}

// Overarching static loop() method for the IODevice subsystem.  Works through the
// list of installed devices and calls their individual _loop() method.
// Devices may not implement this, but it is useful for things like animations 
// or flashing LEDs.
// The current value of micros() is passed as a parameter, so the called loop function
// doesn't need to invoke it.
void IODevice::loop() {
  unsigned long currentMicros = micros();
  // Call every device's loop function in turn.
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    dev->_loop(currentMicros);
  }
}

// Display a list of all the devices on the diagnostic stream.
void IODevice::DumpAll() {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    dev->_display();
  }
}

// Determine if the specified vpin is allocated to a device.
bool IODevice::exists(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) return true;
  }
  return false;
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
  DIAG(F("Unknown device VPins:%d-%d"), (int)_firstID, (int)_firstID+_nPins-1);
}

// Find device and pass configuration values on to it.  Return false if not found.
bool IODevice::configure(VPIN vpin, int paramCount, int params[]) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) {
      // Found appropriate object
      return dev->_configure(vpin, paramCount, params);
    }
  }
  return false;
}

// Write value to virtual pin(s).  If multiple devices are allocated the same pin
//  then only the first one found will be used.
void IODevice::write(VPIN vpin, int value) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) {
      dev->_write(vpin, value);
      return;
    }
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::write(): VPin ID %d not found!"), (int)vpin);
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


//==================================================================================================================
// Instance members
//------------------------------------------------------------------------------------------------------------------

// Method to check whether the id corresponds to this device
bool IODevice::owns(VPIN id) {
  return (id >= _firstID && id < _firstID + _nPins);
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
  DIAG(F("IODevice::write(): VPin ID %d not found!"), (int)vpin);
#endif  
} 

// Read value from virtual pin.
bool IODevice::read(VPIN vpin) {
  for (IODevice *dev = _firstDevice; dev != 0; dev = dev->_nextDevice) {
    if (dev->owns(vpin)) 
      return dev->_read(vpin);
  }
#ifdef DIAG_IO
  DIAG(F("IODevice::read(): VPin %d not found!"), (int)vpin);
#endif
  return false;
}

bool IODevice::_isDeletable() {
  return false;
}

// Start of chain of devices.
IODevice *IODevice::_firstDevice = 0;

// Start of chain of device types.
IODeviceType *IODevice::_firstDeviceType = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

// Constructor
ArduinoPins::ArduinoPins(VPIN firstID, int nPins) {
  _firstID = firstID;
  _nPins = nPins;
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
  pinMode(pin, INPUT_PULLUP);
  int value = digitalRead(pin);
  #ifdef DIAG_IO
  //DIAG(F("Arduino Read Pin:%d Value:%d"), pin, value);
  #endif
  return value;
}

void ArduinoPins::_display() {
  DIAG(F("Arduino VPins:%d-%d"), (int)_firstID, (int)_firstID+_nPins-1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

