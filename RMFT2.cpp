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
#include <Arduino.h>
#include "RMFT2.h"
#include "DCC.h"
#include "DIAG.h"
#include "WiThrottle.h"
#include "DCCEXParser.h"
#include "Sensors.h"
#include "Turnouts.h"
#include "Outputs.h"


// Command parsing keywords
const int16_t HASH_KEYWORD_RMFT=17997;    
const int16_t HASH_KEYWORD_ON = 2657;
const int16_t HASH_KEYWORD_SCHEDULE=-9179;
const int16_t HASH_KEYWORD_RESERVE=11392;
const int16_t HASH_KEYWORD_FREE=-23052;
const int16_t HASH_KEYWORD_LATCH=1618;  
const int16_t HASH_KEYWORD_UNLATCH=1353;
const int16_t HASH_KEYWORD_PAUSE=-4142;
const int16_t HASH_KEYWORD_RESUME=27609;
const int16_t HASH_KEYWORD_STATUS=-25932;

// One instance of RMFT clas is used for each "thread" in the automation.
// Each thread manages a loco on a journey through the layout, and/or may manage a scenery automation.
// The thrrads exist in a ring, each time through loop() the next thread in the ring is serviced.

// Statics 
int16_t RMFT2::progtrackLocoId;  // used for callback when detecting a loco on prograck
bool RMFT2::diag=false;      // <D RMFT ON>  
RMFT2 * RMFT2::loopTask=NULL; // loopTask contains the address of ONE of the tasks in a ring.
RMFT2 * RMFT2::pausingTask=NULL; // Task causing a PAUSE. 
 // when pausingTask is set, that is the ONLY task that gets any service,
 // and all others will have their locos stopped, then resumed after the pausing task resumes.
byte RMFT2::flags[MAX_FLAGS];

/* static */ void RMFT2::begin() { 
  DCCEXParser::setRMFTFilter(RMFT2::ComandFilter);
  for (int f=0;f<MAX_FLAGS;f++) flags[f]=0;
  int pcounter;
  // first pass startup, define any turnouts or servos and count size.
  for (pcounter=0;; pcounter+=2){
     byte opcode=GETFLASH(RMFT2::RouteCode+pcounter);
     if (opcode==OPCODE_ENDEXRAIL) break;
     if (opcode==OPCODE_TURNOUT) {
      byte id=GETFLASH(RMFT2::RouteCode+pcounter+1);
      pcounter+=2;
      byte addr=GETFLASH(RMFT2::RouteCode+pcounter+1);
      pcounter+=2;
      byte subAddr=GETFLASH(RMFT2::RouteCode+pcounter+1);
      Turnout::createDCC(id,addr,subAddr);
     }
  } 
  pcounter+=2; // include ENDROUTES opcode
  DIAG(F("RMFT myAutomation=%db, MAX_FLAGS=%d"), pcounter,MAX_FLAGS);
  new RMFT2(0); // add the startup route
}

// This filter intercepst <> commands to do the following:
// - Implement RMFT specific commands/diagnostics 
// - Reject/modify JMRI commands that would interfere with RMFT processing 
void RMFT2::ComandFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]) {
    (void)stream; // avoid compiler warning if we don't access this parameter 
    bool reject=false;
    switch(opcode) {
        
     case 'D':
        if (p[0]==HASH_KEYWORD_RMFT) { // <D RMFT ON/OFF>
           diag = paramCount==2 && (p[1]==HASH_KEYWORD_ON || p[1]==1);
           opcode=0;
        }
        break;

      case 't': // THROTTLE <t [REGISTER] CAB SPEED DIRECTION>          
          // TODO - Monitor throttle commands and reject any that are in current automation
          break;
          
     case '/':  // New RMFT command
          reject=!parseSlash(stream,paramCount,p);
          opcode=0;
          break;
          
     default:  // other commands pass through 
     break;       
   }
 if (reject) {
   opcode=0;
   StringFormatter::send(stream,F("<X>"));
   }
}
     
bool RMFT2::parseSlash(Print * stream, byte & paramCount, int16_t p[]) {
          
          switch (p[0]) {
            case HASH_KEYWORD_PAUSE: // </ PAUSE>
                 if (paramCount!=1) return false;
                 DCC::setThrottle(0,1,true);  // pause all locos on the track         
                 pausingTask=(RMFT2 *)1; // Impossible task address
                 return true;
                 
            case HASH_KEYWORD_RESUME: // </ RESUME>
                 if (paramCount!=1) return false;
                 pausingTask=NULL;
                 {
                  RMFT2 * task=loopTask;                 
                  while(task) {
                      if (task->loco) task->driveLoco(task->speedo);
                      task=task->next;      
                      if (task==loopTask) break;      
                    }
                 }        
                 return true;
                 
            case HASH_KEYWORD_STATUS: // </STATUS>
                 if (paramCount!=1) return false;
                 StringFormatter::send(stream, F("<* RMFT STATUS"));
                 {
                  RMFT2 * task=loopTask;
                  while(task) {
                      StringFormatter::send(stream,F("\nPC=%d,DT=%d,LOCO=%d%c,SPEED=%d%c"),
                            task->progCounter,task->delayTime,task->loco,
                            task->invert?'I':' ',
                            task->speedo, 
                            task->forward?'F':'R'
                            );
                      task=task->next;      
                      if (task==loopTask) break;      
                    }                            
                 }
                 // Now stream the flags 
                 for (int id=0;id<MAX_FLAGS; id++) {
                   byte flag=flags[id];
                   if (flag) {
                     StringFormatter::send(stream,F("\nflags[%d} "),id);
                     if (flag & SECTION_FLAG) StringFormatter::send(stream,F(" RESERVED"));
                     if (flag & SENSOR_FLAG) StringFormatter::send(stream,F(" SET"));
                     }                 
                 }
                 StringFormatter::send(stream,F(" *>\n"));
                 return true;
                 
            case HASH_KEYWORD_SCHEDULE: // </ SCHEDULE [cab] route >
                 if (paramCount<2 || paramCount>3) return false;
                 {
                  byte route=(paramCount==2) ? p[1] : p[2];
                  uint16_t cab=(paramCount==2)? 0 : p[1];                 
                  new RMFT2(route,cab);                    
                 }
              return true;
                 
            default:
              break;
          }
          
          // all other / commands take 1 parameter 0 to MAX_FLAGS-1     

          if (paramCount!=2 || p[1]<0  || p[1]>=MAX_FLAGS) return false;

          switch (p[0]) {     
            case HASH_KEYWORD_RESERVE:  // force reserve a section
                 setFlag(p[1],SECTION_FLAG);
                 return true;
    
            case HASH_KEYWORD_FREE:  // force free a section
                 setFlag(p[1],0,SECTION_FLAG);
                 return true;
                
            case HASH_KEYWORD_LATCH:
                 setFlag(p[1], SENSOR_FLAG);
                 return true;
   
            case HASH_KEYWORD_UNLATCH:
                 setFlag(p[1], 0, SENSOR_FLAG);
                 return true;
                  
            default:
                 return false;                 
          }
    }


// This emits Routes and Automations to Withrottle
// Automations are given a state to set the button to "handoff" which implies 
// handing over the loco to the automation.
// Routes are given "Set" buttons and do not cause the loco to be handed over. 
void RMFT2::emitWithrottleRouteList(Print* stream) {
  bool first=true;
  for (int pcounter=0;;pcounter+=2) {
    byte opcode=GETFLASH(RMFT2::RouteCode+pcounter);
    if (opcode==OPCODE_ENDEXRAIL) break;
    if (opcode==OPCODE_ROUTE || opcode==OPCODE_AUTOMATION) {
      byte  route=GETFLASH(RMFT2::RouteCode+pcounter+1);
      if (first) {
        first=false;
        StringFormatter::send(stream,F("PRT]\\[Routes}|{Route]\\[Set}|{2]\\[Handoff}|{4\nPRL"));
      }
      if (opcode==OPCODE_AUTOMATION) StringFormatter::send(stream,F("]\\[A%d}|{Auto_%d}|{4"), route, route);
      else                           StringFormatter::send(stream,F("]\\[R%d}|{Route_%d}|{2"), route, route);      
    }
  }
  if (!first) StringFormatter::send(stream,F("\n"));  // end of route list 
}

// An instance of this object guides a loco through a journey, or simply animates something. 
RMFT2::RMFT2(byte route, uint16_t cab) : RMFT2(locateRouteStart(route)) {
  loco=cab;
}

RMFT2::RMFT2(int progCtr) {
  progCounter=progCtr;
  delayTime=0;
  loco=0;
  speedo=0;
  forward=true;
  invert=false;
  stackDepth=0;
   
  // chain into ring of RMFTs
  if (loopTask==NULL) {
    loopTask=this;
    next=this;
  }
  else {
        next=loopTask->next;
        loopTask->next=this;
  }
  
  if (diag) DIAG(F("RMFT(%d)"),progCounter);
}


RMFT2::~RMFT2() {
  if (next==this) loopTask=NULL;
  else for (RMFT2* ring=next;;ring=ring->next) if (ring->next == this) {
           ring->next=next;
           loopTask=next;
           break;
       }
}


int RMFT2::locateRouteStart(short _route) {
  if (_route==0) return 0; // Route 0 is always start of ROUTES for default startup 
  for (int pcounter=0;;pcounter+=2) {
    byte opcode=GETFLASH(RMFT2::RouteCode+pcounter);
    if (opcode==OPCODE_ENDEXRAIL) return -1;
    if ((opcode==OPCODE_ROUTE || opcode==OPCODE_AUTOMATION || opcode==OPCODE_SEQUENCE) 
       &&  _route==GETFLASH(RMFT2::RouteCode+pcounter+1)) return pcounter;
  }
  return -1;
}


void RMFT2::driveLoco(byte speed) {
     if (loco<0) return;  // Caution, allows broadcast! 
     if (diag) DIAG(F("RMFT drive %d %d %d"),loco,speed,forward^invert);
     DCC::setThrottle(loco,speed, forward^invert);
     speedo=speed;
     // TODO... if broadcast speed 0 then pause all other tasks. 
}

bool RMFT2::readSensor(short id) {
  if (getFlag(id,SENSOR_FLAG)) return true; // latched on
  short s= IODevice::read(id);
  if (s==1 && diag) DIAG(F("RMFT Sensor %d hit"),id);
  return s==1;
}

bool RMFT2::skipIfBlock() {
  // returns false if killed
  short nest = 1;
  while (nest > 0) {
    progCounter += 2;
    byte opcode =  GETFLASH(RMFT2::RouteCode+progCounter);;
    switch(opcode) {
      case OPCODE_ENDEXRAIL: 
           kill(F("missing ENDIF"), nest);
           return false;  
      case OPCODE_IF:
      case OPCODE_IFNOT:
      case OPCODE_IFRANDOM:
           nest++;
           break;
      case OPCODE_ENDIF:
           nest--;
           break;
      default:
      break;
    }
  }
  return true; 
}



/* static */ void RMFT2::readLocoCallback(int cv) {
     progtrackLocoId=cv;
}

void RMFT2::loop() {
  
  // Round Robin call to a RMFT task each time 
     if (loopTask==NULL) return; 
     
     loopTask=loopTask->next;
 
     if (pausingTask==NULL || pausingTask==loopTask) loopTask->loop2();
}    

  
void RMFT2::loop2() {
   if (delayTime!=0 && millis()-delayStart < delayTime) return;
     
  byte opcode = GETFLASH(RMFT2::RouteCode+progCounter);
  byte operand =  GETFLASH(RMFT2::RouteCode+progCounter+1);
   
  // Attention: Returning from this switch leaves the program counter unchanged.
  //            This is used for unfinished waits for timers or sensors.
  //            Breaking from this switch will step to the next step in the route. 
  switch (opcode) {
    
    case OPCODE_THROW:
         Turnout::activate(operand, true);
         break;
          
    case OPCODE_CLOSE:
         Turnout::activate(operand, false);
         break; 
    
    case OPCODE_REV:
      forward = false;
      driveLoco(operand);
      break;
    
    case OPCODE_FWD:
      forward = true;
      driveLoco(operand);
      break;
      
    case OPCODE_SPEED:
      driveLoco(operand);
      break;
    
    case OPCODE_INVERT_DIRECTION:
      invert= !invert;
      driveLoco(speedo);
      break;
      
    case OPCODE_RESERVE:
      if (getFlag(operand,SECTION_FLAG)) {
        driveLoco(0);
        delayMe(500);
        return;
      }
      setFlag(operand,SECTION_FLAG);
      break;
    
    case OPCODE_FREE:
      setFlag(operand,0,SECTION_FLAG);
      break;
    
    case OPCODE_AT:
      if (readSensor(operand)) break;
      delayMe(50);
      return;
    
    case OPCODE_AFTER: // waits for sensor to hit and then remain off for 0.5 seconds. (must come after an AT operation)
      if (readSensor(operand)) {
        // reset timer to half a second and keep waiting
        waitAfter=millis();
        return; 
      }
      if (millis()-waitAfter < 500 ) return;   
      break;
    
    case OPCODE_LATCH:
      setFlag(operand,SENSOR_FLAG);
      break;
    
    case OPCODE_UNLATCH:
      setFlag(operand,0,SENSOR_FLAG);
      break;

    case OPCODE_SET:
      {
        Output * o=Output::get(operand);
        if (o) o->activate(true); 
      }
      break;
  
    case OPCODE_RESET:
      {
        Output * o=Output::get(operand);
        if (o) o->activate(false); 
      }
      break;
    
    case OPCODE_PAUSE:
         DCC::setThrottle(0,1,true);  // pause all locos on the track
         pausingTask=this;
         break;

    case OPCODE_POM:
        if (loco==0) break;
        {
          int cv=getIntOperand(operand);
          byte bValue=GETFLASH(RMFT2::RouteCode+progCounter+3);
          DCC::writeCVByteMain(loco, cv, bValue);
        }        
        break;

    case OPCODE_RESUME:
         pausingTask=NULL;
         driveLoco(speedo);
         for (RMFT2 * t=next; t!=this;t=t->next) if (t->loco >0) t->driveLoco(t->speedo);
          break;        
    
    case OPCODE_IF: // do next operand if sensor set
      if (!readSensor(operand)) if (!skipIfBlock()) return;
      break;
    
    case OPCODE_IFNOT: // do next operand if sensor not set
      if (readSensor(operand)) if (!skipIfBlock()) return;
      break;
   
    case OPCODE_IFRANDOM: // do block on random percentage
      if (random(100)>=operand) if (!skipIfBlock()) return;
      break;
    
    case OPCODE_ENDIF:
      break;
    
    case OPCODE_DELAY:
      delayMe(getIntOperand(operand)*100);
      break;
   
    case OPCODE_DELAYMINS:
      delayMe(operand*60*1000);
      break;
    
    case OPCODE_RANDWAIT:
      delayMe((long)random(getIntOperand(operand)*10));
      break;
    
    case OPCODE_RED:
      doSignal(operand,true,false,false);
      break;
    
    case OPCODE_AMBER:
      doSignal(operand,false,true,false);
      break;
    
    case OPCODE_GREEN:
      doSignal(operand,false,false,true);
      break;
       
    case OPCODE_FON:      
      DCC::setFn(loco,operand,true);
      break;
    
    case OPCODE_FOFF:
      DCC::setFn(loco,operand,false);
      break;

    case OPCODE_FOLLOW:
      progCounter=locateRouteStart(operand);
      if (progCounter<0) kill(F("FOLLOW unknown"), operand); 
      return;
  
    case OPCODE_CALL:
      if (stackDepth==MAX_STACK_DEPTH) {
        kill(F("CALL stack"), stackDepth);
        return;
      }
      callStack[stackDepth++]=progCounter;
      progCounter=locateRouteStart(operand);
      if (progCounter<0) kill(F("CALL unknown"),operand); 
      return;

    case OPCODE_RETURN:
      if (stackDepth==0) {
        kill(F("RETURN stack"));
        return;
      }
      progCounter=callStack[--stackDepth];
      return;
      
    case OPCODE_ENDTASK:
    case OPCODE_ENDEXRAIL:
      kill();
      return;
      
    case OPCODE_JOIN:
       DCC::setProgTrackSyncMain(true);
       break;

    case OPCODE_UNJOIN:
       DCC::setProgTrackSyncMain(false);
       break;
       
    case OPCODE_READ_LOCO1: // READ_LOCO is implemented as 2 separate opcodes
       DCC::getLocoId(readLocoCallback);
       break;
      
      case OPCODE_READ_LOCO2:
       if (progtrackLocoId<0) {
        delayMe(100);
        return; // still waiting for callback
       }
       loco=progtrackLocoId;
       speedo=0;
       forward=true;
       invert=false;
       break;
       
       case OPCODE_SCHEDULE:
           {
            // Create new task and transfer loco.....
            // but cheat by swapping prog counters with new task  
            new RMFT2(operand);
            int swap=loopTask->progCounter;
            loopTask->progCounter=progCounter+2;
            progCounter=swap;
           }
           break;
       
       case OPCODE_SETLOCO:
           {
             loco=getIntOperand(operand); 
             speedo=0;
             forward=true;
             invert=false;
             if (diag) DIAG(F("RMFT SETLOCO %d"),loco);
            }
       break;
       
       case OPCODE_ROUTE:
          if (diag) DIAG(F("RMFT ROUTE(%d)"),operand);
          break;

       case OPCODE_AUTOMATION:
          if (diag) DIAG(F("RMFT AUTOMATION(%d)"),operand);
          break;

       case OPCODE_SEQUENCE:
          if (diag) DIAG(F("RMFT SEQUENCE(%d)"),operand);
          break;

       case OPCODE_PAD: // Just a padding for previous opcode needing >1 operad byte.
       case OPCODE_SIGNAL: // Signal definition ignore at run time
       case OPCODE_TURNOUT: // Turnout definition ignored at runtime
       case OPCODE_ONCLOSE: // Turnout event catcers ignored here
       case OPCODE_ONTHROW: // Turnout definition ignored at runtime
       break;
    
    default:
      DIAG(F("RMFT Opcode %d not supported at offset %d"),opcode,progCounter);
    }
    // Falling out of the switch means move on to the next opcode
    progCounter+=2;
}

void RMFT2::delayMe(long delay) {
     delayTime=delay;
     delayStart=millis();
}

void RMFT2::setFlag(byte id,byte onMask, byte offMask) {  
   if (FLAGOVERFLOW(id)) return; // Outside UNO range limit
   byte f=flags[id];
   f &= ~offMask;
   f |= onMask;
}

byte RMFT2::getFlag(byte id,byte mask) {
   if (FLAGOVERFLOW(id)) return 0; // Outside UNO range limit
   return flags[id]&mask;   
}

void RMFT2::kill(const FSH * reason, int operand) {
     if (reason) DIAG(F("RMFT ERROR at pc=%d, cab=%d, %S %d"), progCounter,loco, reason, operand);
     else if (diag) DIAG(F("ENDTASK at pc=%d"), progCounter); 
     delete this;
}

int RMFT2::getIntOperand(byte operand1) {
   // second part of long operands follow 3 bytes after the current progCounter
   int operand2 =  GETFLASH(RMFT2::RouteCode+progCounter+3);
   progCounter+=2; // Skip the OPCODE_PAD and its operand. 
   return (operand1<<7) | operand2;
}

/* static */ void RMFT2::doSignal(byte id,bool red, bool amber, bool green) { 
  for (int pcounter=0;; pcounter+=2){
     byte opcode=GETFLASH(RMFT2::RouteCode+pcounter);
     if (opcode==OPCODE_ENDEXRAIL) return;
     if (opcode!=OPCODE_SIGNAL) continue;
     byte redpin=GETFLASH(RMFT2::RouteCode+pcounter+1);
     if (redpin!=id)continue;
     pcounter+=2;
     byte amberpin=GETFLASH(RMFT2::RouteCode+pcounter+1);
     pcounter+=2;
     byte greenpin=GETFLASH(RMFT2::RouteCode+pcounter+1);
     IODevice::write(redpin,red);
     if (amberpin) IODevice::write(amberpin,amber);
     if (greenpin) IODevice::write(amberpin,green);
     return;
   }
  } 
 void RMFT2::turnoutEvent(int id, bool thrown) {
    byte huntFor=thrown? OPCODE_ONTHROW : OPCODE_ONCLOSE;
    for (int pcounter=0;; pcounter+=2){
     byte opcode=GETFLASH(RMFT2::RouteCode+pcounter);
     if (opcode==OPCODE_ENDEXRAIL) return;
     if (opcode!=huntFor) continue;
     if (id!=GETFLASH(RMFT2::RouteCode+pcounter+1)) continue;
     new RMFT2(pcounter);  // new task starts at this instruction
     return;
   }
  }
