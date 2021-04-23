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
 * Turnout data is stored in the the following form:
 * DCC Turnouts:
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bits 6 to 0=0 (not used)
 *  Byte subAddress (1-4)
 *  Word address (0-)
 * 
 * Servo Turnouts:
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bit 6=1
 *                Bits 5 to 0=PWM Pin (0-63)
 *  Byte positionByte: Eight LSBs of activePosition
 *  Word positionWord: Bits 15-12=Four MSBs of activePosition
 *                     Bits 11-0=inactivePosition
 * 
 * LCN Turnouts
 *  Word ID
 *  Byte tStatus: Bit 7=active
 *                Bits 6 to 0=0
 *  Byte subAddress: Not used
 *  Word address: 0xffff (-1)
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
const int  VPIN_TURNOUT_ADDRESS=-2;      // spoof dcc address -2 indicates a VPIN turnout

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
  static Turnout *create(int id, VPIN vpin);
  static Turnout *createDCC(int id , int address , int subAddress);
  static Turnout *createServo(int id , byte vpin , int activeAngle, int inactiveAngle, int profile=1);
  static Turnout *create(int id, int params, int16_t p[]);
  static Turnout *create(int id);
  void activate(bool state);
  static void printAll(Print *);
  void print(Print *stream);
#ifdef EESTOREDEBUG
  static void print(Turnout *tt);
#endif
}; // Turnout
  
#endif
