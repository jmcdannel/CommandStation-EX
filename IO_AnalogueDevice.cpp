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

void Analogue::create(VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile, uint8_t initialState) {
  #ifdef DIAG_IO
  DIAG(F("Analogue Create VPIN:%d State:%d"), vpin, initialState);
  #endif
  IODevice::remove(vpin); // Delete any existing device that may conflict
  Analogue *dev = new Analogue(vpin, activePosition, inactivePosition, profile);
  if (dev) {
    dev->_state = initialState;
    dev->_configure(vpin, activePosition, inactivePosition, profile);
    addDevice(dev);
  }
}

//==================================================================================================================
// Instance members
//------------------------------------------------------------------------------------------------------------------

Analogue::Analogue(VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile) {
  _firstVpin = vpin;
  _activePosition = activePosition;
  _inactivePosition = inactivePosition;
  _profile = (ProfileType)profile;
  _nPins = 1;
  _numSteps = _stepNumber = 0;
  _state = -1; // Unknown state
}

void Analogue::_configure(VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile) {
  #ifdef DIAG_IO
  DIAG(F("Analogue configure Vpin:%d %d-%d %d"), vpin, activePosition, inactivePosition, profile);
  #endif
  _activePosition = activePosition;
  _inactivePosition = inactivePosition;
  _currentPosition = _state ? activePosition : inactivePosition;
  _profile = (ProfileType)profile;
  
  IODevice::writeDownstream(vpin, _currentPosition);
  _stepNumber = _numSteps = 0;  // this forces next call to updatePosition to switch off the servo
}

// Periodically update current position if it is changing.
// It's not worth going faster than 20ms as this is the pulse 
// frequency for the PWM Servo driver.  50ms is acceptable.
void Analogue::_loop(unsigned long currentMicros) {
  unsigned int currentTime = currentMicros; // 16-bit low part (up to 65ms)
  if (currentTime - _lastRefreshTime >= refreshInterval*1000) { 
    updatePosition();
    _lastRefreshTime = currentTime;
  }
}

// Device params are activePosition, inactivePosition, profile and state.
bool Analogue::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_SERVO) return false;
  if (paramCount != 4) return false;
  _state = params[3];  // Set initial state
  _configure(vpin, params[0], params[1], (ProfileType)params[2]);
  return true;
}

void Analogue::_write(VPIN vpin, int value) {
  #ifdef DIAG_IO
  DIAG(F("Analogue Write Vpin:%d Value:%d"), vpin, value);
  #else
  (void)vpin;  // suppress compiler warning
  #endif
  if (value) value = 1;
  if (_state == -1) {
    // Initial position being set, go straight there
    _fromPosition = _toPosition = _currentPosition = (_state ? _activePosition : _inactivePosition);
    _state = value;
    updatePosition();
    return;
  }

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
  DIAG(F("Analogue Vpin:%d Range:%d,%d"), 
    _firstVpin, _activePosition, _inactivePosition);
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
    writeDownstream(_firstVpin, _currentPosition);
  } else if (_stepNumber < _numSteps + _catchupSteps) {
    // We've finished animation, wait a little to allow servo to catch up
    _stepNumber++;
  } else if (_stepNumber == _numSteps + _catchupSteps 
            && _currentPosition != 4095 && _currentPosition != 0) {
    // Then switch off PWM to prevent annoying servo buzz
    writeDownstream(_firstVpin, 0);
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
