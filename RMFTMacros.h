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
#ifndef RMFTMacros_H
#define RMFTMacros_H

// The entire automation script is contained within a byte arrayRMFT2::RouteCode[]
// made up of opcode and parameter pairs.
// The array is normally built using the macros below as this makes it easier 
// to manage the cases where:
// - padding must be applied to ensure the correct alignment of the next instruction
// - large parameters must be split up
// - multiple parameters aligned correctly
// - a single macro requires multiple operations 

#define I_SPLIT(val) (val)>>7,OPCODE_PAD,(val)&0x7F

#define EXRAIL const  FLASH  byte RMFT2::RouteCode[] = {
#define AUTOMATION(id)  OPCODE_AUTOMATION, id, 
#define ROUTE(id)  OPCODE_ROUTE, id, 
#define SEQUENCE(id)  OPCODE_SEQUENCE, id, 
#define ENDTASK OPCODE_ENDTASK,0,
#define ENDEXRAIL OPCODE_ENDTASK,0,OPCODE_ENDEXRAIL,0 };
 
#define AFTER(sensor_id) OPCODE_AT,sensor_id,OPCODE_AFTER,sensor_id,
#define AMBER(signal_id) OPCODE_AMBER,signal_id,
#define AT(sensor_id) OPCODE_AT,sensor_id,
#define CALL(route) OPCODE_CALL,route,
#define DELAY(mindelay) OPCODE_DELAY,I_SPLIT(mindelay),
#define DELAYMINS(mindelay) OPCODE_DELAYMINS,mindelay,
#define DELAYRANDOM(mindelay,maxdelay) OPCODE_DELAY,I_SPLIT(mindelay),OPCODE_RANDWAIT,I_SPLIT(maxdelay-mindelay),
#define ENDIF  OPCODE_ENDIF,0,
#define FOFF(func) OPCODE_FOFF,func,
#define FOLLOW(route) OPCODE_FOLLOW,route,
#define FON(func) OPCODE_FON,func,
#define FREE(blockid) OPCODE_FREE,blockid,
#define FWD(speed) OPCODE_FWD,speed,
#define GREEN(signal_id) OPCODE_GREEN,signal_id,
#define IF(sensor_id) OPCODE_IF,sensor_id,
#define IFNOT(sensor_id) OPCODE_IFNOT,sensor_id,
#define IFRANDOM(percent) OPCODE_IFRANDOM,percent,
#define INVERT_DIRECTION OPCODE_INVERT_DIRECTION,0,
#define JOIN OPCODE_JOIN,0,
#define LATCH(sensor_id) OPCODE_LATCH,sensor_id,
#define PAUSE OPCODE_PAUSE,0,
#define READ_LOCO OPCODE_READ_LOCO1,0,OPCODE_READ_LOCO2,0,
#define RED(signal_id) OPCODE_RED,signal_id,
#define RESERVE(blockid) OPCODE_RESERVE,blockid,
#define RESET(sensor_id) OPCODE_RESET,sensor_id,
#define RESUME OPCODE_RESUME,0,
#define RETURN OPCODE_RETURN,0,
#define REV(speed) OPCODE_REV,speed,
#define SCHEDULE(route) OPCODE_SCHEDULE,route,
#define SERVO(id,position,speed) OPCODE_SERVO,id,OPCODE_PAD,position/4,OPCODE_PAD,speed,
#define SETLOCO(loco) OPCODE_SETLOCO,I_SPLIT(loco),
#define SET(sensor_id) OPCODE_SET,sensor_id,
#define SPEED(speed) OPCODE_SPEED,speed,
#define STOP OPCODE_SPEED,0, 
#undef SIGNAL
#define SIGNAL(redpin,amberpin,greenpin) OPCODE_SIGNAL,redpin,OPCODE_PAD,amberpin,OPCODE_PAD,greenpin, 
#define TURNOUT(id,addr,subaddr) OPCODE_TURNOUT,id,OPCODE_PAD,addr,OPCODE_PAD,subaddr,
#define UNJOIN OPCODE_UNJOIN,0,
#define ESTOP OPCODE_SPEED,1, 
#define THROW(id)  OPCODE_THROW,id,
#define CLOSE(id)  OPCODE_CLOSE,id,
#define UNLATCH(sensor_id) OPCODE_UNLATCH,sensor_id,
   
// The following are the operation codes (or instructions) for a kind of virtual machine.
// Each instruction is normally 2 bytes long with an operation code followed by a parameter.
// In cases where more than one parameter is required, the first parameter is followed by one  
// or more OPCODE_PAD instructions with the subsequent parameters. This wastes a byte but makes 
// searching easier as a parameter can never be confused with an opcode. 
// 
enum OPCODE {OPCODE_THROW,OPCODE_CLOSE,
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
             OPCODE_JOIN,OPCODE_UNJOIN,OPCODE_READ_LOCO1,OPCODE_READ_LOCO2,
             OPCODE_SCHEDULE,OPCODE_SETLOCO,
             OPCODE_PAUSE, OPCODE_RESUME, 
             OPCODE_ROUTE,OPCODE_AUTOMATION,OPCODE_SEQUENCE,OPCODE_ENDTASK,OPCODE_ENDEXRAIL
             };


#endif
