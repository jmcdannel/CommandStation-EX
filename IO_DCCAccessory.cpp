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

#include "DCC.h"
#include "IODevice.h"
#include "DIAG.h"

void DCCAccessoryDecoder::create(VPIN vpin, int DCCAddress, int DCCSubaddress) {
  IODevice::remove(vpin);
  DCCAccessoryDecoder *dev = new DCCAccessoryDecoder();
  dev->_firstID = vpin;
  dev->_nPins = 1;
  dev->_DCCAddress = DCCAddress;
  dev->_DCCSubaddress= DCCSubaddress;
  addDevice(dev);
}

void DCCAccessoryDecoder::create(VPIN vpin, int DCCLinearAddress) {
  int DCCAddress = (DCCLinearAddress-1)/4;
  int DCCSubaddress = DCCLinearAddress - DCCAddress*4;
  create(vpin, DCCAddress, DCCSubaddress);
}

// Constructor
DCCAccessoryDecoder::DCCAccessoryDecoder() {}

// Device-specific write function.
void DCCAccessoryDecoder::_write(VPIN id, int state) {
  int subaddress = id-_firstID+_DCCSubaddress;
  #ifdef DIAG_IO
  DIAG(F("DCC Write Addr:%d Subddr:%d State:%d"), _DCCAddress, subaddress, state);
  #endif
  DCC::setAccessory(_DCCAddress, subaddress, state);
}

void DCCAccessoryDecoder::_display() {
  DIAG(F("DCC VPins:%d-%d Addr:%d/%d "), 
    (int)_firstID, (int)_firstID+_nPins-1, (int)_DCCAddress, (int)_DCCSubaddress);
}

