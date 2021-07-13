/*
 *  © 2020,2021 Chris Harlow. All rights reserved.
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
#ifndef RMFTMacros_H
#define RMFTMacros_H

// The entire automation script is contained within a byte array RMFT2::RouteCode[]
// made up of opcode and parameter pairs.
// ech opcode is a 1 byte operation plus 2 byte operand. 
// The array is normally built using the macros below as this makes it easier 
// to manage the cases where:
// - padding must be applied to ensure the correct alignment of the next instruction
// - large parameters must be split up
// - multiple parameters aligned correctly
// - a single macro requires multiple operations 

#define V(val) (val)&0x7F,(val)>>7
#define NOP 0,0
#define B(val) val,0

#define EXRAIL const  FLASH  byte RMFT2::RouteCode[] = {
#define AUTOMATION(id)  OPCODE_AUTOMATION, V(id), 
#define ROUTE(id)  OPCODE_ROUTE, V(id), 
#define SEQUENCE(id)  OPCODE_SEQUENCE, V(id), 
#define ENDTASK OPCODE_ENDTASK,NOP,
#define DONE OPCODE_ENDTASK,NOP,
#define ENDEXRAIL OPCODE_ENDTASK,NOP,OPCODE_ENDEXRAIL,NOP };
 
#define AFTER(sensor_id) OPCODE_AT,V(sensor_id),OPCODE_AFTER,V(sensor_id),
#define AMBER(signal_id) OPCODE_AMBER,V(signal_id),
#define AT(sensor_id) OPCODE_AT,V(sensor_id),
#define CALL(route) OPCODE_CALL,V(route),
#define CLOSE(id)  OPCODE_CLOSE,V(id),
#define DELAY(mindelay) OPCODE_DELAY,V(mindelay),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,V(mindelay),
#define DELAYRANDOM(mindelay,maxdelay) OPCODE_DELAY,V(mindelay),OPCODE_RANDWAIT,V(maxdelay-mindelay),
#define ENDIF  OPCODE_ENDIF,NOP,
#define ESTOP OPCODE_SPEED,V(1), 
#define FOFF(func) OPCODE_FOFF,V(func),
#define FOLLOW(route) OPCODE_FOLLOW,V(route),
#define FON(func) OPCODE_FON,V(func),
#define FREE(blockid) OPCODE_FREE,V(blockid),
#define FWD(speed) OPCODE_FWD,V(speed),
#define GREEN(signal_id) OPCODE_GREEN,V(signal_id),
#define IF(sensor_id) OPCODE_IF,V(sensor_id),
#define IFNOT(sensor_id) OPCODE_IFNOT,V(sensor_id),
#define IFRANDOM(percent) OPCODE_IFRANDOM,V(percent),
#define INVERT_DIRECTION OPCODE_INVERT_DIRECTION,NOP,
#define JOIN OPCODE_JOIN,NOP,
#define LATCH(sensor_id) OPCODE_LATCH,V(sensor_id),
#define ONCLOSE(turnout_id) OPCODE_ONCLOSE,V(turnout_id),
#define ONTHROW(turnout_id) OPCODE_ONTHROW,V(turnout_id),
#define PAUSE OPCODE_PAUSE,NOP,
#define POM(cv,value) OPCODE_POM,V(cv),OPCODE_PAD,V(value),
#define READ_LOCO OPCODE_READ_LOCO1,NOP,OPCODE_READ_LOCO2,NOP,
#define RED(signal_id) OPCODE_RED,V(signal_id),
#define RESERVE(blockid) OPCODE_RESERVE,V(blockid),
#define RESET(sensor_id) OPCODE_RESET,V(sensor_id),
#define RESUME OPCODE_RESUME,NOP,
#define RETURN OPCODE_RETURN,NOP,
#define REV(speed) OPCODE_REV,V(speed),
#define START(route) OPCODE_START,V(route),
#define SERVO(id,position,profile) OPCODE_SERVO,V(id),OPCODE_PAD,V(position),OPCODE_PAD,V(profile),
#define SETLOCO(loco) OPCODE_SETLOCO,V(loco),
#define SET(sensor_id) OPCODE_SET,V(sensor_id),
#define SPEED(speed) OPCODE_SPEED,V(speed),
#define STOP OPCODE_SPEED,V(0), 
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) OPCODE_SIGNAL,V(redpin),OPCODE_PAD,V(amberpin),OPCODE_PAD,V(greenpin), 
#define SERVO_TURNOUT(pin,activeAngle,inactiveAngle) OPCODE_SERVOTURNOUT,V(pin),OPCODE_PAD,V(actibeAngle),OPCODE
#define PIN_TURNOUT(pin) OPCODE_PINTURNOUT,V(pin), 
#define THROW(id)  OPCODE_THROW,V(id),
#define TURNOUT(id,addr,subaddr) OPCODE_TURNOUT,V(id),OPCODE_PAD,V(addr),OPCODE_PAD,V(subaddr),
#define UNJOIN OPCODE_UNJOIN,NOP,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,V(sensor_id),
   
// The following are the operation codes (or instructions) for a kind of virtual machine.
// Each instruction is normally 2 bytes long with an operation code followed by a parameter.
// In cases where more than one parameter is required, the first parameter is followed by one  
// or more OPCODE_PAD instructions with the subsequent parameters. This wastes a byte but makes 
// searching easier as a parameter can never be confused with an opcode. 
// 
enum OPCODE : byte {OPCODE_THROW,OPCODE_CLOSE,
             OPCODE_FWD,OPCODE_REV,OPCODE_SPEED,OPCODE_INVERT_DIRECTION,
             OPCODE_RESERVE,OPCODE_FREE,
             OPCODE_AT,OPCODE_AFTER,
             OPCODE_LATCH,OPCODE_UNLATCH,OPCODE_SET,OPCODE_RESET,
             OPCODE_IF,OPCODE_IFNOT,OPCODE_ENDIF,OPCODE_IFRANDOM,
             OPCODE_DELAY,OPCODE_DELAYMINS,OPCODE_RANDWAIT,
             OPCODE_FON,OPCODE_FOFF,
             OPCODE_RED,OPCODE_GREEN,OPCODE_AMBER,
             OPCODE_SERVO,OPCODE_SIGNAL,OPCODE_TURNOUT,
             OPCODE_PAD,OPCODE_FOLLOW,OPCODE_CALL,OPCODE_RETURN,
             OPCODE_JOIN,OPCODE_UNJOIN,OPCODE_READ_LOCO1,OPCODE_READ_LOCO2,OPCODE_POM,
             OPCODE_START,OPCODE_SETLOCO,
             OPCODE_PAUSE, OPCODE_RESUME,
             OPCODE_ONCLOSE, OPCODE_ONTHROW, OPCODE_SERVOTURNOUT, OPCODE_PINTURNOUT,  
             OPCODE_ROUTE,OPCODE_AUTOMATION,OPCODE_SEQUENCE,OPCODE_ENDTASK,OPCODE_ENDEXRAIL
             };
#endif
