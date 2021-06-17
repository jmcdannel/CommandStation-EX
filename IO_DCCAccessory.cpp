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

void DCCAccessoryDecoder::create(VPIN vpin, int nPins, int DCCAddress, int DCCSubaddress) {
  int linearAddress = (DCCAddress << 2) + DCCSubaddress;
  create(vpin, nPins, linearAddress);
}

void DCCAccessoryDecoder::create(VPIN vpin, int nPins, int DCCLinearAddress) {
  IODevice::remove(vpin);
  DCCAccessoryDecoder *dev = new DCCAccessoryDecoder(vpin, nPins, DCCLinearAddress);
  addDevice(dev);
}

// Constructor
DCCAccessoryDecoder::DCCAccessoryDecoder(VPIN vpin, int nPins, int DCCLinearAddress) {
   _firstVpin = vpin;
  _nPins = nPins;
  _dccLinearAddress = DCCLinearAddress;
  int endAddress = DCCLinearAddress + _nPins - 1;
  DIAG(F("DCC Accessory Decoder configured Vpins:%d-%d Linear Address:%d-%d (%d/%d-%d/%d)"), _firstVpin, _firstVpin+_nPins-1,
      _dccLinearAddress, _dccLinearAddress+_nPins-1,
      _dccLinearAddress>>2, _dccLinearAddress%4, endAddress>>2, endAddress%4);
}

// Device-specific write function.
void DCCAccessoryDecoder::_write(VPIN id, int state) {
  int linearAddress = _dccLinearAddress + id - _firstVpin;
  #ifdef DIAG_IO
  DIAG(F("DCC Write Linear Address:%d State:%d"), linearAddress, state);
  #endif
  DCC::setAccessory(linearAddress >> 2, linearAddress % 4, state);
}

void DCCAccessoryDecoder::_display() {
  int endAddress = _dccLinearAddress + _nPins - 1;
  DIAG(F("DCC Accessory Vpins:%d-%d Linear Address:%d-%d (%d/%d-%d/%d)"), _firstVpin, _firstVpin+_nPins-1,
      _dccLinearAddress, _dccLinearAddress+_nPins-1,
      _dccLinearAddress>>2, _dccLinearAddress%4, endAddress>>2, endAddress%4);
}

