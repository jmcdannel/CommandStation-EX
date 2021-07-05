/*
 *  © 2020, Chris Harlow. All rights reserved.
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
#ifndef RMFT2_H
#define RMFT2_H
#include "FSH.h"
#include "RMFTMacros.h"
 
  // Flag bits for status of hardware and TPL
  static const short SECTION_FLAG = 0x01;
  static const short SENSOR_FLAG = 0x02;

  static const byte  MAX_STACK_DEPTH=4;
 
#if (defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAMD_ZERO))
   static const short MAX_FLAGS=256;
   #define FLAGOVERFLOW(x) false
#else
  static const short MAX_FLAGS=64;
  #define FLAGOVERFLOW(x) x>=MAX_FLAGS
#endif

 class RMFT2 {
   public:
    static void begin();
    static void loop();
    RMFT2(byte route, uint16_t cab=0);
    ~RMFT2();
    static void readLocoCallback(int cv);
    static void emitWithrottleRouteList(Print* stream);   
private: 
    static void ComandFilter(Print * stream, byte & opcode, byte & paramCount, int p[]);
    static bool parseSlash(Print * stream, byte & paramCount, int p[]) ;
    static void streamFlags(Print* stream);
    static void setFlag(byte id,byte onMask, byte OffMask=0);
    static byte getFlag(byte id,byte mask);   
    static int locateRouteStart(short _route);
    static int progtrackLocoId;
    static void doSignal(byte id,bool red, bool amber, bool green); 

    static RMFT2 * loopTask;
    static RMFT2 * pausingTask;
    void delayMe(long millisecs);
    void driveLoco(byte speedo);
    bool readSensor(short id);
    bool skipIfBlock();
    bool readLoco();
    void showManual();
    void showProg(bool progOn);
    bool doManual();
    void loop2();
    void kill(const FSH * reason=NULL,int operand=0);          
    int  getIntOperand(byte operand1);

   static bool diag;
   static const  FLASH  byte RouteCode[];
   static byte flags[MAX_FLAGS];
 
 // Local variables - exist for each instance/task 
    RMFT2 *next;   // loop chain 
    int progCounter;    // Byte offset of next route opcode in ROUTES table
    unsigned long delayStart; // Used by opcodes that must be recalled before completing
    unsigned long waitAfter; // Used by OPCODE_AFTER
    unsigned long  delayTime;
    int loco;
    bool forward;
    bool invert;
    int speedo;
    byte stackDepth;
    int callStack[MAX_STACK_DEPTH];
};
#endif
