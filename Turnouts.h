/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
 * Turnout data is stored in a 6 byte structure in the following form:
 *
 * DCC Turnouts:
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bits 6 to 0=zeroes (not used)
 *  Byte subAddress: Bits 7-5: zeroes (not used)
 *                Bits 4-0: DCC Sub Address (1-4)
 *  Word address: Bits 15-0: DCC Address (0-65535)
 * 
 * Servo Turnouts:
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bit 6=one
 *                Bits 5 to 0=PWM Pin (0-63)
 *  Byte positionByte: Eight LSBs of activePosition (0-511)
 *  Word positionWord: Bits 15-13=zeroes
 *                     Bits 12-10=Profile
 *                     Bit 9=MSB of activePosition
 *                     Bits 8-0=inactivePosition (0-511)
 * 
 * LCN Turnouts
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bits 6 to 0=zero (not used)
 *  Byte subAddress: Bits 7-0=zeroes (not used)
 *  Word address: Bits 15-0=0xffff (-1)
 *
 * VPIN Turnouts (digital outputs on Arduino or extender)
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bits 6 to 0=zeroes (not used)
 *  Byte subAddress: Bits 7-0: 0xFE (-1)
 *  Word address: Bits 15-0=VPIN number
 * 
 */

#ifndef Turnouts_h
#define Turnouts_h

#include <Arduino.h>
#include "DCC.h"
#include "LCN.h"
#include "IODevice.h"

const byte STATUS_ACTIVE=0x80; // Flag as activated
const byte STATUS_PWM=0x40; // Flag as a PWM turnout
const byte STATUS_PWMPIN=0x3F; // PWM  pin 0-63
const int  LCN_TURNOUT_ADDRESS=-1;  // spoof dcc address -1 indicates a LCN turnout
const int  VPIN_TURNOUT_SUBADDRESS=-2;  // spoof dcc subaddress -2 indicates a VPIN turnout

struct TurnoutData {
  int id;
  uint8_t tStatus; // has STATUS_ACTIVE, STATUS_PWM, STATUS_PWMPIN  
  union {
    struct {
      // DCC subaddress (1-4) or VPIN
      uint8_t subAddress;
      // DCC address, or -1 (LCN) or -2 (VPIN)
      int address;
    };
    struct {
      // Least significant 8 bits of activePosition
      uint8_t positionByte;
      // Most significant 4 bits of activePosition, and inactivePosition.
      uint16_t positionWord;
    };
  }; // DCC address or PWM servo positions 
};

class Turnout {
public:
  static Turnout *firstTurnout;
  static int turnoutlistHash;
  TurnoutData data;
  Turnout *nextTurnout;
  static  bool activate(int n, bool state);
  static Turnout* get(int);
  static bool remove(int);
  static bool isActive(int);
  static void load();
  static void store();
  static Turnout *createVpin(int id, VPIN vpin, int initialState=0);
  static Turnout *createDCC(int id , int address , int subAddress);
  static Turnout *createServo(int id , VPIN vpin , uint16_t activeAngle, uint16_t inactiveAngle, uint8_t profile=1, uint8_t initialState=0);
  static Turnout *create(int id, int params, int16_t p[]);
  static Turnout *create(int id);
  void activate(bool state);
  static void printAll(Print *);
  void print(Print *stream);
#ifdef EESTOREDEBUG
  static void print(Turnout *tt);
#endif
private:
  int num;  // EEPROM address of tStatus in TurnoutData struct, or zero if not stored.
}; // Turnout
  
#endif
