/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *
 *  This file is part of CommandStation-EX
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

#include <stdarg.h>
#include "I2CManager.h"
#include "DIAG.h"

#if defined(ARDUINO_ARCH_AVR) 
#include "MiniWire.h"  // Cut-down Wire library with asynchronous transmission
#else
#include <Wire.h>    // Standard Wire library
#define MiniWire Wire
#endif

// If not already initialised, initialise I2C (wire).
void I2CManagerClass::begin(void) {
  if (!_beginCompleted) {
    _beginCompleted = true;
    MiniWire.begin();

    // Probe and list devices.
    bool found = false;
    for (byte addr=1; addr<127; addr++) {
      if (exists(addr)) {
        found = true; 
        DIAG(F("I2C Device found at x%x"), addr);
      }
    }
    if (!found) DIAG(F("No I2C Devices found"));
  }
}

// Set clock speed to the lowest requested one. If none requested,
//  the Wire default is 100kHz.
void I2CManagerClass::setClock(uint32_t speed) {
  if (speed < _clockSpeed && !_clockSpeedFixed) {
    _clockSpeed = speed;
  }
  MiniWire.setClock(_clockSpeed);
}

// Force clock speed to that specified.  It can then only 
// be overridden by calling MiniWire.setClock directly.
void I2CManagerClass::forceClock(uint32_t speed) {
  if (!_clockSpeedFixed) {
    _clockSpeed = speed;
    _clockSpeedFixed = true;
    MiniWire.setClock(_clockSpeed);
  }
}

// Check if specified I2C address is responding.
// Returns 0 if OK, or error code.
uint8_t I2CManagerClass::checkAddress(uint8_t address) {
  MiniWire.beginTransmission(address);
  return MiniWire.endTransmission();
}

// Write a complete transmission to I2C using a supplied buffer of data
uint8_t I2CManagerClass::write(uint8_t address, const uint8_t buffer[], uint8_t size) {
  MiniWire.beginTransmission(address);
  MiniWire.write(buffer, size);
#if !defined(MiniWire)   // MiniWire is defined as a macro if Wire is in use
  // Write without waiting for completion
  return MiniWire.endTransmission(true, true);
#else
  return Wire.endTransmission();
#endif
}

// Write a complete transmission to I2C using a supplied buffer of data in Flash
uint8_t I2CManagerClass::write_P(uint8_t address, const uint8_t buffer[], uint8_t size) {
  uint8_t ramBuffer[size];
  const uint8_t *p1 = buffer;
  for (uint8_t i=0; i<size; i++)
    ramBuffer[i] = GETFLASH(p1++);
  return write(address, ramBuffer, size);
}

// Write a complete transmission to I2C using a list of data 
uint8_t I2CManagerClass::write(uint8_t address, int nBytes, ...) {
  uint8_t buffer[nBytes];
  va_list args;
  va_start(args, nBytes);
  for (uint8_t i=0; i<nBytes; i++)
    buffer[i] = va_arg(args, int);
  va_end(args);
  return write(address, buffer, nBytes);
}

// Write a command and read response, returns number of bytes received.
//  Different modules use different ways of accessing registers: 
//    PCF8574 I/O expander justs needs the address (no data);
//    PCA9685 needs a two byte command to select the register(s) to be read;
//    MCP23016/7 need a one-byte command to select the register.
// Some devices use 8-bit registers exclusively and some have 16-bit registers.
// Therefore the following function is general purpose, to apply to any
// type of I2C device.
//
uint8_t I2CManagerClass::read(uint8_t address, uint8_t readBuffer[], uint8_t readSize,
                              uint8_t writeBuffer[], uint8_t writeSize) {
  if (writeSize > 0) {
    MiniWire.beginTransmission(address);
    MiniWire.write(writeBuffer, writeSize);
    MiniWire.endTransmission(false); // Don't free bus yet
  }
  MiniWire.requestFrom(address, (size_t)readSize);
  uint8_t nBytes = 0;
  while (MiniWire.available() && nBytes < readSize) 
    readBuffer[nBytes++] = MiniWire.read();
  return nBytes;
}

// Overload of read() to allow command to be specified as a series of bytes.
uint8_t I2CManagerClass::read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
                                  uint8_t writeSize, ...) {
  va_list args;
  // Copy the series of bytes into an array.
  va_start(args, writeSize);
  uint8_t writeBuffer[writeSize];
  for (uint8_t i=0; i<writeSize; i++)
    writeBuffer[i] = va_arg(args, int);
  va_end(args);
  return read(address, readBuffer, readSize, writeBuffer, writeSize);
}

I2CManagerClass I2CManager = I2CManagerClass();