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

#ifndef IO_GPIOBASE_H
#define IO_GPIOBASE_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

class GPIOBase : public IODevice {

protected:
  // Constructor
  GPIOBase(FSH *deviceName, VPIN firstVpin, uint8_t nPins, uint8_t I2CAddress, int interruptPin);
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration function.  
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
  // Pin write function.
  void _write(VPIN vpin, int value);
  // Pin read function.
  int _read(VPIN vpin);
  void _display();
  void _loop(unsigned long currentMicros);

  // Data fields
  uint8_t _I2CAddress; 
  // Allocate enough space for 16 input pins
  uint16_t _portInputState; 
  uint16_t _portOutputState;
  uint16_t _portMode;
  uint16_t _portPullup;
  uint16_t _portInvert;  // Inversion mask for inputs
  // Interval between refreshes of each input port
  static const int _portTickTime = 4000;
  unsigned long _lastLoopEntry = 0;

  // Virtual functions for interfacing with I2C GPIO Device
  virtual void _writeGpioPort() = 0;
  virtual void _readGpioPort(bool immediate=true) = 0;
  virtual void _writePullups() {};
  virtual void _writePortModes() {};
  virtual void _setupDevice() {};
  virtual void _processCompletion(uint8_t status) {
    (void)status; // Suppress compiler warning
  };

  I2CRB requestBlock;
  FSH *_deviceName;
};

#endif