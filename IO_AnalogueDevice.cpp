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

#include "IO_AnalogueDevice.h"
#include "FSH.h"
#include "DIAG.h"

//==================================================================================================================
// Static members
//------------------------------------------------------------------------------------------------------------------
IODevice *Analogue::createInstance(VPIN vpin) {
  IODevice::remove(vpin); // Delete any existing device that may conflict
  Analogue *dev = new Analogue();
  dev->_firstID = vpin;
  dev->_nPins = 1;
  dev->_numSteps = dev->_stepNumber = 0;
  dev->_state = 0;
  addDevice(dev);
  return dev; 
}

void Analogue::create(VPIN vpin, VPIN devicePin, int activePosition, int inactivePosition, int profile) {
  Analogue *dev = (Analogue *)createInstance(vpin);
  dev->_configure(vpin, devicePin, activePosition, inactivePosition, profile);
}

//==================================================================================================================
// Instance members
//------------------------------------------------------------------------------------------------------------------
void Analogue::_configure(VPIN vpin, VPIN devicePin, int activePosition, int inactivePosition, int profile) {
  (void)vpin;    // Suppress compiler warning
  #ifdef DIAG_IO
  DIAG(F("Analogue configure Vpin:%d->Vpin:%d %d-%d %d"), vpin, devicePin, activePosition, inactivePosition, profile);
  #endif
  _devicePin = devicePin;
  _activePosition = activePosition;
  _currentPosition = _inactivePosition = inactivePosition;
  _profile = (ProfileType)profile;
  
  IODevice::writeDownstream(vpin, _inactivePosition);
  _stepNumber = _numSteps = 0;  // this forces next call to updatePosition to switch off the servo
}

// Periodically update current position if it is changing.
// It's not worth going faster than 20ms as this is the pulse 
// frequency for the PWM Servo driver.  50ms is acceptable.
void Analogue::_loop(unsigned long currentMicros) {
  unsigned int currentTime = currentMicros/1000; // 16-bit millisecond timer
  if (currentTime - _lastRefreshTime >= refreshInterval) { 
    updatePosition();
    _lastRefreshTime = currentTime;
  }
}

// Device params are devicePin, activePosition, inactivePosition, and profile.
bool Analogue::_configure(VPIN vpin, int paramCount, int params[]) {
  (void)vpin;  // Suppress compiler warning
  if (paramCount != 4) return false;
  _configure(vpin, params[0], params[1], params[2], (ProfileType)params[3]);
  return true;
}

void Analogue::_write(VPIN vpin, int value) {
  #ifdef DIAG_IO
  DIAG(F("Analogue Write VPin:%d Value:%d"), vpin, value);
  #else
  (void)vpin;  // suppress compiler warning
  #endif
  if (value) value = 1;
  if (_state == value) return; // Nothing to do.
  switch (_profile) {
    case Instant: 
      _numSteps = 1;
      break;
    case Fast:
      _numSteps = 10;
      break;
    case Medium:
      _numSteps = 20;
      break;
    case Slow:
      _numSteps = 40;
      break;      
    case Bounce:
      _numSteps = sizeof(profile);
      break;
    default:
      _profile = Fast;
      _numSteps = 10;
      break;
  }
  _state = value;
  _stepNumber = 0;
  // Move from the current position, which may be in the 
  //  middle of an animation.
  _fromPosition = _currentPosition;
  _toPosition = _state ? _activePosition : _inactivePosition;
  updatePosition();
  _lastRefreshTime = millis();
}

void Analogue::_display() {
  DIAG(F("Analogue VPin:%d->VPin:%d Range:%d,%d"), 
    _firstID, _devicePin, _activePosition, _inactivePosition);
}

// Private function to reposition servo
// TODO: Could calculate step number from elapsed time, to allow for erratic loop timing.
void Analogue::updatePosition() {
  if (_stepNumber > _numSteps + _catchupSteps) return; // No animation in progress
  bool changed = false;
  switch (_profile) {
    case Instant:
    case Fast:
    case Medium:
    case Slow:
      if (_stepNumber < _numSteps) {
        _stepNumber++;
        _currentPosition = map(_stepNumber, 0, _numSteps, _fromPosition, _toPosition);
        changed = true;
      }
      break;
    case Bounce:
      if (_stepNumber < _numSteps) {
        byte profileValue = GETFLASH(&profile[_stepNumber]);
        _currentPosition = map(profileValue, 0, 100, _fromPosition, _toPosition);
        changed = true;
        _stepNumber++;
      }
      break;
    default:
      break;
  }
  // Write to PWM module.  Use writeDownstream.
  if (changed) {
    IODevice::writeDownstream(_devicePin, _currentPosition);
  } else if (_stepNumber < _numSteps + _catchupSteps) {
    // We've finished animation, wait a little to allow servo to catch up
    _stepNumber++;
  } else if (_stepNumber == _numSteps + _catchupSteps 
            && _currentPosition != 4095 && _currentPosition != 0) {
    // Then switch off PWM to prevent annoying servo buzz
    IODevice::writeDownstream(_devicePin, 0);
    _stepNumber++;
  }
}

bool Analogue::_isDeletable() {
  return true;
}

// Profile for a bouncing signal or turnout
// The profile below is in the range 0-100% and should be combined with the desired limits
// of the servo set by _activePosition and _inactivePosition.  The profile is symmetrical here,
// i.e. the bounce is the same on the down action as on the up action.
const byte FLASH Analogue::profile[30] = 
    {2,3,7,13,33,50,83,100,83,75,70,65,60,60,65,74,84,100,83,75,70,70,72,75,80,87,92,97,100,100};
