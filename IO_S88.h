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

/*
 * The S88 Bus is essentially a shift register consisting of one or more 
 * S88 modules, connected one to another in one long chain.  The register
 * is read by the following steps:
 *   1) The LOAD/PS line goes to HIGH, then the CLK line is pulsed HIGH.  This tells
 *      the registers to acquire data from the latched parallel inputs.
 *   2) With LOAD/PS still high, the RESET signal is pulsed HIGH, which clears 
 *      the parallel input upstream latches.
 *   3) With LOAD/PS LOW, the shift is performed by pulsing the CLK line high.  On each
 *      pulse, the data in the shift register is presented, bit by bit, to the
 *      DATA-IN line.
 * 
 * Example configuration in mySetup.cpp:
 *    #include "IO_S88.h"
 *    void mySetup() {
 *      IO_S88::create(2500, 16, 40,41,42,43);
 *    }
 * 
 * This creates an S88 bus instance with 16 inputs (VPINs 2500-2515).  The 
 * LOAD pin is 40, RESET is 41, CLK is 42 and DATA is 43.  These four pins 
 * must be local GPIO pins (not on an I/O expander).
 * 
 * Depending on the S88 modules being used and the cabling, the timing of the
 * interface may have to be adjusted.  There are three parameters below that 
 * affect this. 
 * 
 * _pulseDelayTime determines how much extra time is allowed 
 * between pulse transitions, to allow for propagation times.
 * 
 * _bitsReadPerLoopEntry determines the number of bits read in each loop entry;
 * if the pulse delay time is increased, the loop entry will take longer, so
 * bitsReadPerLoopEntry should be reduced to compensate.
 * 
 * _acquireCycleTime determines the minimum time between successive acquire cycles
 * of the inputs on the S88 bus.  Bear in mind that the Sensor code includes 
 * anti-bounce logic which means that fleeting state changes may be ignored,
 * so reducing the acquireCycleTime may not have the desired effect.
 * 
 */

#ifndef IO_S88_H
#define IO_S88_H

#include "IODevice.h"

class IO_S88 : public IODevice {

private:
  uint8_t _loadPin;
  uint8_t _resetPin;
  uint8_t _clockPin;
  uint8_t _dataInPin;
  uint8_t *_values;  // Bit array, initialised in _begin method.
  int _currentByteIndex;
  unsigned long _lastAquireCycleStart;

  // The following constants are 'finger-in-the-air' as there is no S88 spec for timing. 
  // The propagation delays in the hardware are theoretically less than a microsecond
  // for cable lengths up to 30 metres.  For slow hardware, the pulseDelayTime may have to be
  // increased.  If so, then reduce bitsReadPerLoopEntry to ensure that the loop doesn't impact
  // on the DCC signal generation.
  const unsigned long _acquireCycleTime = 20000; // 20 milliseconds between acquire cycles
  const uint8_t _pulseDelayTime = 2;  // Delay microsecs; can be tuned for the S88 hardware
  const uint8_t _bitsReadPerLoopEntry = 32;  // 2 x 32 x 2us = 128us per loop entry.  Must be multiple of 8.

public:
  IO_S88(VPIN firstVpin, int nPins, uint8_t loadPin, uint8_t resetPin, uint8_t clockPin, uint8_t dataInPin) :
    IODevice(firstVpin, nPins),
    _loadPin(loadPin),
    _resetPin(resetPin),
    _clockPin(clockPin),
    _dataInPin(dataInPin),
    _currentByteIndex(0),
    _lastAquireCycleStart(0)
  { 
    // Allocate memory for input values.
    _values = (uint8_t *)calloc((nPins+7)/8, 1);
    addDevice(this);
  }

  static void create(VPIN firstVpin, int nPins, uint8_t loadPin, uint8_t resetPin, uint8_t clockPin, uint8_t dataInPin) {
    new IO_S88(firstVpin, nPins, loadPin, resetPin, clockPin, dataInPin);
  }

protected:
  void _begin() override {
    pinMode(_loadPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_clockPin, OUTPUT);
    pinMode(_dataInPin, INPUT);

    #ifdef DIAG_IO 
    _display();
    #endif
  }

  // Read method returns the latest aquired value for the nominated VPIN number.
  int _read(VPIN vpin) override {
    uint16_t pin = vpin - _firstVpin;
    uint8_t mask = 1 << (pin % 8);
    uint16_t byteIndex = pin / 8;
    return (_values[byteIndex] & mask) ? 1 : 0;
  }

  // Loop method acquires the input states from the shift register.
  // At the beginning of each acquisition cycle, instruct the bus registers to acquire the
  // input states from the latches, then reset the latches.  On this and
  // subsequent loop entries, some of the input states are shifted from the
  // registers, until they have all been read.  Then the whole process
  // resumes for the next acquisition cycle.
  void _loop(unsigned long currentMicros) override {
    // If just starting a new read, then latch the input values into the S88
    // registers.
    if (_currentByteIndex == 0) {
      _lastAquireCycleStart = currentMicros;
      // Set LOAD pin
      ArduinoPins::fastWriteDigital(_loadPin, HIGH);
      delayMicroseconds(_pulseDelayTime);
      // Pulse CLOCK pin to read inputs into registers
      ArduinoPins::fastWriteDigital(_clockPin, HIGH);
      delayMicroseconds(_pulseDelayTime);
      ArduinoPins::fastWriteDigital(_clockPin, LOW);
      delayMicroseconds(_pulseDelayTime);
      // Pulse RESET pin to clear inputs ready for next acquisition period
      ArduinoPins::fastWriteDigital(_resetPin, HIGH);
      delayMicroseconds(_pulseDelayTime);
      ArduinoPins::fastWriteDigital(_resetPin, LOW);
      delayMicroseconds(_pulseDelayTime);
      // Clear LOAD pin.  This leaves the first bit in the DATA-IN line.
      ArduinoPins::fastWriteDigital(_loadPin, LOW);
      delayMicroseconds(_pulseDelayTime);
    }

    // Read each bit in turn from the shiftregister.
    for (uint8_t bitsRead = 0; _currentByteIndex < (_nPins+7)/8  && bitsRead < _bitsReadPerLoopEntry; ) {
      uint8_t currentByteValue = 0;
      uint8_t mask = 1;
      for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
        if (ArduinoPins::fastReadDigital(_dataInPin)) currentByteValue |= mask;
        bitsRead++;
        mask <<= 1;
          // Clock next bit in.
        ArduinoPins::fastWriteDigital(_clockPin, HIGH);
        delayMicroseconds(_pulseDelayTime);
        ArduinoPins::fastWriteDigital(_clockPin, LOW);        
        if (bitsRead < _bitsReadPerLoopEntry)
          delayMicroseconds(_pulseDelayTime);
      }
      _values[_currentByteIndex++] = currentByteValue;
    }
    if (_currentByteIndex >= (_nPins+7)/8) {
      // All bits in the shift register have been read now, so 
      // don't read again until next acquisition cycle time
      _currentByteIndex = 0;  // Reset byte counter.
      delayUntil(_lastAquireCycleStart + _acquireCycleTime);
    }
  }

  void _display() override {
    DIAG(F("S88 Bus Configured on Vpins %d-%d, LOAD=%d RESET=%d CLK=%d DATAIN=%d"), 
      _firstVpin, _firstVpin+_nPins-1, _loadPin, _resetPin, _clockPin, _dataInPin);
  }

};

#endif
