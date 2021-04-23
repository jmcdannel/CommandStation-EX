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
 *  Class Analogue for servos and variable brightness LEDs.  Well, anything
 *  that can be driven by a Pulse Width Modulated signal, really...
 *  In fact, all the class does is converts the values of '1' and 
 *  '0' on the input pin into the values corresponding to 'activePosition' 
 *  and 'inactivePosition' respectively on the output pin, with a 
 *  configurable progression between states.  The device 
 *  attached to the output pin decides whether it's PWM or whatever!
 */

#ifndef analoguedevice_h
#define analoguedevice_h

#include "IODevice.h"
#include "FSH.h"

class Analogue : public IODevice {
public:

  enum ProfileType {
    Instant = 0,  // Moves immediately between positions
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4    // For semaphores/turnouts with a bit of bounce!!
  };

  static void registerDeviceType() {
    _registerDeviceType(DeviceType::Analogue, createInstance);
  }
  static IODevice *createInstance(VPIN vpin);
  static void create(VPIN vpin, VPIN devicePin, int activePosition, int inactivePosition, int profile);  
  Analogue() { }

  void _loop(unsigned long currentMicros);
  void _write(VPIN vpin, int value);
  bool _configure(VPIN vpin, int paramCount, int params[]);
  void _configure(VPIN vpin, VPIN devicePin, int activePosition, int inactivePosition, int profile);
  void _display();
  bool _isDeletable();

 
private:
  // Recalculate output
  void updatePosition();
        
  VPIN _devicePin = VPIN_NONE;
  int _activePosition;
  int _inactivePosition;
  int _currentPosition;
  int _fromPosition;
  int _toPosition;
  int8_t _state = 0;  
  uint8_t _stepNumber = 0;  // current step in animation
  uint8_t _numSteps;  // number of steps in animation
  static const uint8_t _catchupSteps = 5; // number of steps to wait before switching servo off
  static const byte FLASH profile[30];
  enum ProfileType _profile;
  const unsigned int refreshInterval = 50; // refresh every 50ms
  unsigned int _lastRefreshTime; // low 16-bits of millis() count.

};

#endif // analoguedevice_h