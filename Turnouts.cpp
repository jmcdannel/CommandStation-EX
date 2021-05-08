/*
 *  © 2013-2016 Gregg E. Berman
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
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

#include "Turnouts.h"
#include "EEStore.h"
#include "StringFormatter.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

// Keywords used for turnout configuration.
const int16_t HASH_KEYWORD_SERVO=27709;
const int16_t HASH_KEYWORD_DCC=6436;
const int16_t HASH_KEYWORD_VPIN=-415;

enum unit8_t {
  TURNOUT_DCC = 1,
  TURNOUT_SERVO = 2,
  TURNOUT_VPIN = 3,
  TURNOUT_LCN = 4,
};

///////////////////////////////////////////////////////////////////////////////
// Static function to print all Turnout states to stream in form "<H id state>"

void Turnout::printAll(Print *stream){
  for (Turnout *tt = Turnout::firstTurnout; tt != NULL; tt = tt->nextTurnout)
    StringFormatter::send(stream, F("<H %d %d>\n"), tt->data.header.id, (tt->data.header.tStatus & STATUS_ACTIVE)!=0);
} // Turnout::printAll

///////////////////////////////////////////////////////////////////////////////
// Object method to print configuration of one Turnout to stream, in one of the following forms:
//  <H id SERVO vpin activePos inactivePos profile state>
//  <H id LCN state>
//  <H id VPIN vpin state>
//  <H id DCC address subAddress state>

void Turnout::print(Print *stream){
  uint8_t state = ((data.header.tStatus & STATUS_ACTIVE) != 0);
  uint8_t type = data.header.tStatus & STATUS_TYPE;
  switch (type) {
    case TURNOUT_SERVO:
      // Servo Turnout
      StringFormatter::send(stream, F("<H %d SERVO %d %d %d %d %d>\n"), data.header.id, data.servoData.vpin, 
        data.servoData.activePosition, data.servoData.inactivePosition, data.servoData.profile, state);
      break;
    case TURNOUT_LCN:
      // LCN Turnout
      StringFormatter::send(stream, F("<H %d LCN %d>\n"), data.header.id, state);
      break;
    case TURNOUT_VPIN:
      // VPIN Digital output
      StringFormatter::send(stream, F("<H %d VPIN %d %d>\n"), data.header.id, data.vpinData.vpin, state);
      break;
    case TURNOUT_DCC:
      // DCC Turnout
      StringFormatter::send(stream, F("<H %d DCC %d %d %d>\n"), data.header.id, 
        (data.dccAccessoryData.address >> 2), (data.dccAccessoryData.address & 3), state);
      break;
    default:
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to activate/deactivate Turnout with ID 'n'.
//   Returns false if turnout not found.

bool Turnout::activate(int n, bool state){
#ifdef EESTOREDEBUG
  DIAG(F("Turnout::activate(%d,%d)"),n,state);
#endif
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  tt->activate(state);
  turnoutlistHash++;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Static function to check if the Turnout with ID 'n' is activated or not.
// Returns false if turnout not found.

bool Turnout::isActive(int n){
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  return tt->isActive();
}


///////////////////////////////////////////////////////////////////////////////
// Object function to check the status of Turnout is activated or not.

bool Turnout::isActive() {
  return (data.header.tStatus & STATUS_ACTIVE) != 0;
}

///////////////////////////////////////////////////////////////////////////////
// Object function to set the status of Turnout to activated or not.
// Does not send any commands, only modifies the tStatus flag.

void Turnout::setActive(bool value) {
  if (value) 
    data.header.tStatus |= STATUS_ACTIVE;
  else
    data.header.tStatus &= ~STATUS_ACTIVE;
}

///////////////////////////////////////////////////////////////////////////////
// Object method to activate or deactivate the Turnout.  

// activate is virtual here so that it can be overridden by a non-DCC turnout mechanism
void Turnout::activate(bool state) {
#ifdef EESTOREDEBUG
  DIAG(F("Turnout::activate(%d)"),state);
#endif
  uint8_t type = data.header.tStatus & 0x7f;
  if (type == TURNOUT_LCN) {
      // A LCN turnout is transmitted to the LCN master.
      LCN::send('T', data.header.id, state);
      return;   // The tStatus will be updated by a message from the LCN master, later.
  } else {
    if (state)
      data.header.tStatus|=STATUS_ACTIVE;
    else
      data.header.tStatus &= ~STATUS_ACTIVE;
    switch (type) {
      case TURNOUT_SERVO:
        IODevice::write(data.servoData.vpin, state);
        break;
      case TURNOUT_VPIN:
        IODevice::write(data.vpinData.vpin, state);
        break;
      case TURNOUT_DCC:
        DCC::setAccessory((data.dccAccessoryData.address >> 2), (data.dccAccessoryData.address & 3), state);
        break;
    }
    // Save state if stored in EEPROM
    if (EEStore::eeStore->data.nTurnouts > 0 && num > 0) 
      EEPROM.put(num, data.header.tStatus);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to find Turnout object specified by ID 'n'.  Return NULL if not found.

Turnout* Turnout::get(int n){
  Turnout *tt;
  for(tt=firstTurnout;tt!=NULL && tt->data.header.id!=n;tt=tt->nextTurnout);
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function to delete Turnout object specified by ID 'n'.  Return false if not found.

bool Turnout::remove(int n){
  Turnout *tt,*pp=NULL;

  for(tt=firstTurnout;tt!=NULL && tt->data.header.id!=n;pp=tt,tt=tt->nextTurnout);

  if(tt==NULL) return false;
  
  if(tt==firstTurnout)
    firstTurnout=tt->nextTurnout;
  else
    pp->nextTurnout=tt->nextTurnout;

  free(tt);
  turnoutlistHash++;
  return true; 
}

///////////////////////////////////////////////////////////////////////////////
// Static function to load all Turnout definitions from EEPROM
// TODO: Consider transmitting the initial state of the DCC/LCN turnout here.
//  (already done for servo turnouts and VPIN turnouts).

void Turnout::load(){
  struct TurnoutData data;
  Turnout *tt;

  for(int i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    // Save pointer to where tStatus will be within EEPROM
    int tStatusPointer = EEStore::pointer() + offsetof(TurnoutData, header.tStatus);
    // Retrieve header
    EEPROM.get(EEStore::pointer(), data.header);
    EEStore::advance(sizeof(data.header));
    
    int initialState = (data.header.tStatus & STATUS_ACTIVE) != 0;
    uint8_t type = data.header.tStatus & STATUS_TYPE;

    switch (type) {
      case TURNOUT_SERVO:
        EEPROM.get(EEStore::pointer(), data.servoData);
        EEStore::advance(sizeof(data.servoData));
        tt=createServo(data.header.id, data.servoData.vpin, 
          data.servoData.activePosition, data.servoData.inactivePosition, data.servoData.profile, initialState);
        break;
      case TURNOUT_VPIN:
        EEPROM.get(EEStore::pointer(), data.vpinData);
        EEStore::advance(sizeof(data.vpinData));
        tt=createVpin(data.header.id, data.vpinData.vpin, initialState);  // VPIN-based turnout
        break;
      case TURNOUT_DCC:
        EEPROM.get(EEStore::pointer(), data.dccAccessoryData);
        EEStore::advance(sizeof(data.dccAccessoryData));
        tt=createDCC(data.header.id, data.dccAccessoryData.address, initialState); // DCC-based turnout
        break;
      case TURNOUT_LCN:
        EEPROM.get(EEStore::pointer(), data.lcnData);
        EEStore::advance(sizeof(data.lcnData));
        tt=createLCN(data.header.id, initialState);
        break;
    }
    tt->num = tStatusPointer;  // Save pointer to tStatus byte within EEPROM
#ifdef EESTOREDEBUG
    print(tt);
#endif
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to store all Turnout definitions to EEPROM

void Turnout::store(){
  Turnout *tt;

  tt=firstTurnout;
  EEStore::eeStore->data.nTurnouts=0;

  while(tt!=NULL){
#ifdef EESTOREDEBUG
    print(tt);
#endif
    tt->num = EEStore::pointer() + offsetof(TurnoutData, header.tStatus); // Save pointer to tstatus byte within EEPROM
    EEPROM.put(EEStore::pointer(),tt->data.header);
    EEStore::advance(sizeof(tt->data.header));
    switch (tt->data.header.tStatus & STATUS_TYPE) {
      case TURNOUT_DCC:
        EEPROM.put(EEStore::pointer(), tt->data.dccAccessoryData);
        EEStore::advance(sizeof(tt->data.dccAccessoryData));
        break;
      case TURNOUT_SERVO:
        EEPROM.put(EEStore::pointer(), tt->data.servoData);
        EEStore::advance(sizeof(tt->data.servoData));
        break;
      case TURNOUT_VPIN:
        EEPROM.put(EEStore::pointer(), tt->data.vpinData);
        EEStore::advance(sizeof(tt->data.vpinData));
        break;
      case TURNOUT_LCN:
        EEPROM.put(EEStore::pointer(), tt->data.lcnData);
        EEStore::advance(sizeof(tt->data.lcnData));
        break;
    }
    tt=tt->nextTurnout;
    EEStore::eeStore->data.nTurnouts++;
  }

}
///////////////////////////////////////////////////////////////////////////////
// Static function for associating a Turnout id with a virtual pin in IODevice space.
// The actual creation and configuration of the pin must be done elsewhere,
// e.g. in mySetup.h during startup of the CS.

Turnout *Turnout::createVpin(int id, VPIN vpin, uint8_t state){
  if (vpin > VPIN_MAX) return NULL;
  Turnout *tt=create(id);
  tt->data.header.tStatus = TURNOUT_VPIN;;
  tt->data.vpinData.vpin = vpin;
  if (state) tt->data.header.tStatus |= STATUS_ACTIVE;
  IODevice::write(vpin, state);   // Set initial state of output.
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function for creating a DCC-controlled Turnout.

Turnout *Turnout::createDCC(int id, uint16_t add, uint8_t subAdd, uint8_t state){
  if (add > 511 || subAdd > 3) return NULL;
  Turnout *tt=create(id);
  tt->data.header.tStatus = TURNOUT_DCC | (state ? STATUS_ACTIVE : 0);
  tt->data.dccAccessoryData.address = add << 2 | subAdd;
  DCC::setAccessory(add, subAdd, state);    // Transmit initial state
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function for creating a LCN-controlled Turnout.

Turnout *Turnout::createLCN(int id, uint8_t state) {
  Turnout *tt=create(id);
  tt->data.header.tStatus = TURNOUT_LCN | (state ? STATUS_ACTIVE : 0);
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Method for creating a PCA9685 PWM Turnout.  

Turnout *Turnout::createServo(int id, VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile, uint8_t initialState){
  if (activePosition > 511 || inactivePosition > 511 || profile > 4) return NULL;
  // Configure PWM interface device
  int deviceParams[] = {(int)activePosition, (int)inactivePosition, profile, initialState};
  if (!IODevice::configure(vpin, sizeof(deviceParams)/sizeof(deviceParams[0]), deviceParams)) return NULL;

  Turnout *tt=create(id);
  tt->data.header.tStatus = TURNOUT_SERVO | (initialState ? STATUS_ACTIVE : 0);
  tt->data.servoData.vpin = vpin;
  tt->data.servoData.activePosition = activePosition;
  tt->data.servoData.inactivePosition = inactivePosition;
  tt->data.servoData.profile = profile;
  return(tt);
}


///////////////////////////////////////////////////////////////////////////////
// Support for <T id SERVO pin activepos inactive pos profile>
// and <T id DCC address subaddress>
// and <T id VPIN pin>

Turnout *Turnout::create(int id, int params, int16_t p[]) {
  if (p[0] == HASH_KEYWORD_SERVO) { // <T id SERVO n n n n>
    if (params == 5)
      return createServo(id, (VPIN)p[1], (uint16_t)p[2], (uint16_t)p[3], (uint8_t)p[4]);
    else  
      return NULL;
  } else if (p[0]==HASH_KEYWORD_DCC) {
    if (params==3 && p[1]>=0 && p[1]<512 && p[2]>=0 && p[2]<4)  // <T id DCC n n>
      return createDCC(id, p[1], p[2]);
    else if (params==2 && p[1]>=0 && p[1]<512*4)  // <T id DCC nn>
      return createDCC(id, p[1]/4, p[1]%4);
    else
      return NULL;
  } else if (p[0] == HASH_KEYWORD_VPIN) { // <T id VPIN n>
    if (params==2)
      return createVpin(id, p[1]);
    else
      return NULL;
  } else if (params==2) { // <T id n n> for DCC or LCN
    return createDCC(id, p[0], p[1]);
  } else if (params==3) { // legacy <T id n n n> for Servo
    return createServo(id, (VPIN)p[0], (uint16_t)p[1], (uint16_t)p[2]);
  }
  return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// Create basic Turnout object.  The details of what sort of object it is 
//  controlling are not set here.

Turnout *Turnout::create(int id){
  Turnout *tt=get(id);
  if (tt==NULL) { 
     tt=(Turnout *)calloc(1,sizeof(Turnout));
     tt->nextTurnout=firstTurnout;
     firstTurnout=tt;
     tt->data.header.id=id;
  }
  tt->num = 0; // Make sure turnout doesn't get written to EEPROM until store() command.
  turnoutlistHash++;
  return tt;
}

///////////////////////////////////////////////////////////////////////////////
//
// Object method to print debug info about the state of a Turnout object
//
#ifdef EESTOREDEBUG
void Turnout::print(Turnout *tt) {
  tt->print(StringFormatter::diagSerial);
}
#endif

///////////////////////////////////////////////////////////////////////////////
Turnout *Turnout::firstTurnout=NULL;
int Turnout::turnoutlistHash=0; //bump on every change so clients know when to refresh their lists
