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

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

// REGISTER ADDRESSES
static const byte PCA9685_MODE1=0x00;      // Mode Register 
static const byte PCA9685_FIRST_SERVO=0x06;  /** low byte first servo register ON*/
static const byte PCA9685_PRESCALE=0xFE;     /** Prescale register for PWM output frequency */
// MODE1 bits
static const byte MODE1_SLEEP=0x10;   /**< Low power mode. Oscillator off */
static const byte MODE1_AI=0x20;      /**< Auto-Increment enabled */
static const byte MODE1_RESTART=0x80; /**< Restart enabled */

static const float FREQUENCY_OSCILLATOR=25000000.0; /** Accurate enough for our purposes  */
static const uint8_t PRESCALE_50HZ = (uint8_t)(((FREQUENCY_OSCILLATOR / (50.0 * 4096.0)) + 0.5) - 1);
static const uint32_t MAX_I2C_SPEED = 1000000L; // PCA9685 rated up to 1MHz I2C clock speed

// Predeclare helper function
static void writeRegister(byte address, byte reg, byte value);

// Create device driver.  This function assumes that one or more PCA9685s will be installed on 
// successive I2C addresses with a contiguous range of VPINs.  For example, the first PCA9685 may
// be at address 0x40 and allocated pins 100-115.  In this case, pins 116-131 would be on another
// PCA9685 on address 0x41, pins 132-147 on address 0x42, and pins 148-163 on address 0x43.  
//
void PCA9685::create(VPIN firstVpin, int nPins, uint8_t I2CAddress) {
  PCA9685 *dev = new PCA9685(firstVpin, nPins, I2CAddress);
  if (dev) addDevice(dev);
}

// Configure a port on the PCA9685.
bool PCA9685::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_SERVO) return false;
  if (paramCount != 4) return false;
  #ifdef DIAG_IO
  DIAG(F("PCA9685 Configure VPIN:%d Apos:%d Ipos:%d Profile:%d state:%d"), 
    vpin, params[0], params[1], params[2], params[3]);
  #endif

  int8_t pin = vpin - _firstVpin;
  struct ServoData *s = _servoData[pin];
  if (!s) { 
    _servoData[pin] = (struct ServoData *)calloc(1, sizeof(struct ServoData));
    s = _servoData[pin];
    if (!s) return false; // Check for failed memory allocation
  }

  s->activePosition = params[0];
  s->currentPosition = s->inactivePosition = params[1];
  s->profile = params[2];

  // Position servo to initial state
  s->state = -1; // Set unknown state, to force reposition
  _write(vpin, params[3]);

  return true;
}

// Constructor
PCA9685::PCA9685(VPIN firstVpin, int nPins, uint8_t I2CAddress) {
  _firstVpin = firstVpin;
  _nPins = min(nPins, 16);
  _I2CAddress = I2CAddress;
  for (int i=0; i<_nPins; i++) {
    _servoData[i] = NULL;
    // struct ServoData *s = _servoData[i];
    // // Assume default servo bounds of 1ms-2ms pulse length
    // s->activePosition = 409;
    // s->inactivePosition = 205;
    // // Set default position to mid-range
    // s->currentPosition = 307;
    // s->profile = Instant;
    // s->numSteps = 0;
    // s->state = -1;  // Current state unknown.
  }

  // Initialise structure used for setting pulse rate
  requestBlock.setWriteParams(_I2CAddress, outputBuffer, sizeof(outputBuffer));
  I2CManager.begin();
  I2CManager.setClock(1000000); // Nominally able to run up to 1MHz on I2C
          // In reality, other devices including the Arduino will limit 
          // the clock speed to a lower rate.

  // Initialise I/O module(s) here.
  if (I2CManager.exists(_I2CAddress)) {
    DIAG(F("PCA9685 I2C:%x configured Vpins:%d-%d"), _I2CAddress, _firstVpin, _firstVpin+_nPins-1);
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_SLEEP | MODE1_AI);    
    writeRegister(_I2CAddress, PCA9685_PRESCALE, PRESCALE_50HZ);   // 50Hz clock, 20ms pulse period.
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_AI);
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_RESTART | MODE1_AI);
    // In theory, we should wait 500us before sending any other commands to each device, to allow
    // the PWM oscillator to get running.  However, we don't do any specific wait, as there's 
    // plenty of other stuff to do before we will send a command.
  }
}

// Device-specific initialisation
void PCA9685::_begin() {
}

// Device-specific write function, invoked from IODevice::write().
void PCA9685::_write(VPIN vpin, int value) {
  #ifdef DIAG_IO
  DIAG(F("PCA9685 Write Vpin:%d Value:%d"), vpin, value);
  #else
  (void)vpin;  // suppress compiler warning
  #endif
  int pin = vpin - _firstVpin;
  if (value) value = 1;

  struct ServoData *s = _servoData[pin];
  if (!s) return;               // Pin not configured
  if (s->state == value) return; // Nothing to do.

  uint8_t profile = s->profile;
  // If current position not known, go straight to selected position.
  if (s->state == -1) profile = Instant;

  // Animated profile.  Initiate the appropriate action.
  s->numSteps = profile==Fast ? 10 : 
                profile==Medium ? 20 : 
                profile==Slow ? 40 : 
                profile==Bounce ? sizeof(_bounceProfile) : 
                1;
  s->state = value;
  s->stepNumber = 0;

  // Update new from/to positions to initiate or change animation.
  s->fromPosition = s->currentPosition;
  s->toPosition = s->state ? s->activePosition : s->inactivePosition;
}

void PCA9685::_loop(unsigned long currentMicros) {
  unsigned int currentTime = (unsigned int) currentMicros;
  if (currentTime - _lastRefreshTime >= refreshInterval * 1000) {
    for (int pin=0; pin<_nPins; pin++) {
      updatePosition(pin);
    }
    _lastRefreshTime = currentMicros;
  }
}

// Private function to reposition servo
// TODO: Could calculate step number from elapsed time, to allow for erratic loop timing.
void PCA9685::updatePosition(uint8_t pin) {
  struct ServoData *s = _servoData[pin];
  if (!s) return;
  if (s->numSteps == 0) return; // No animation in progress
  if (s->stepNumber < s->numSteps) {
    s->stepNumber++;
    if (s->profile == Bounce) {
      // Retrieve step positions from array in flash
      byte profileValue = GETFLASH(&_bounceProfile[s->stepNumber]);
      s->currentPosition = map(profileValue, 0, 100, s->fromPosition, s->toPosition);
    } else {
      // All other profiles - calculate step by linear interpolation between from and to positions.
      s->currentPosition = map(s->stepNumber, 0, s->numSteps, s->fromPosition, s->toPosition);
    }
    // Send servo command
    writeDevice(pin, s->currentPosition);
  } else if (s->stepNumber < s->numSteps + _catchupSteps) {
    // We've finished animation, wait a little to allow servo to catch up
    s->stepNumber++;
  } else if (s->stepNumber == s->numSteps + _catchupSteps 
            && s->currentPosition != 4095 && s->currentPosition != 0) {
    // Switch off PWM to prevent annoying servo buzz
    writeDevice(pin, 0);
    s->numSteps = 0;  // Done now.
  }
}

// writeDevice takes a pin in range 0 to _nPins-1 within the device, and a value
// between 0 and 4095 for the PWM mark-to-period ratio, with 4095 being 100%.
void PCA9685::writeDevice(uint8_t pin, int value) {
  #ifdef DIAG_IO
  DIAG(F("PCA9685 I2C:x%x WriteDevice Pin:%d Value:%d"), _I2CAddress, pin, value);
  #endif
  // Wait for previous request to complete
  requestBlock.wait();
  // Set up new request.
  outputBuffer[0] = PCA9685_FIRST_SERVO + 4 * pin;
  outputBuffer[1] = 0;
  outputBuffer[2] = (value == 4095 ? 0x10 : 0);  // 4095=full on
  outputBuffer[3] = value & 0xff;
  outputBuffer[4] = value >> 8;
  I2CManager.queueRequest(&requestBlock);
}

// Display details of this device.
void PCA9685::_display() {
  DIAG(F("PCA9685 I2C:x%x Vpins:%d-%d"), _I2CAddress, (int)_firstVpin, 
    (int)_firstVpin+_nPins-1);
}

// Internal helper function for this device
static void writeRegister(byte address, byte reg, byte value) {
  I2CManager.write(address, 2, reg, value);
}

// Profile for a bouncing signal or turnout
// The profile below is in the range 0-100% and should be combined with the desired limits
// of the servo set by _activePosition and _inactivePosition.  The profile is symmetrical here,
// i.e. the bounce is the same on the down action as on the up action.  First entry isn't used.
const byte FLASH PCA9685::_bounceProfile[30] = 
    {0,2,3,7,13,33,50,83,100,83,75,70,65,60,60,65,74,84,100,83,75,70,70,72,75,80,87,92,97,100};