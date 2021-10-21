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

/*
 * ENC28J60 default mode of operation:
 *
 * The node number is used as the last byte of the MAC address
 * field, purely so that the MAC address is unique for each distinct 
 * node number.  The ENC28J60 driver doesn't implement ARP cache, 
 * since it would take up a minimum of 11 bytes per node and, with
 * up to 255 nodes, this would take a significant part of the RAM.
 * Instead, all transmissions are sent with MAC address allOnes, i.e.
 * as a broadcast.  Regular sensor state updates are broadcast anyway, 
 * and other write/writeAnalogue command packets are relatively 
 * infrequent.
 *
 * Usage:
 *  DCCNetDriver driver = new ENC28J60Driver(10);
 * 
 * The ENC28J60 device has to be connected to the hardware MISO, MOSI, SCK and CS pins of the 
 * microcontroller.  The CS pin is specified in the command above (e.g. 10).
 * 
 */

#ifndef NET_ENC28J60_H
#define NET_ENC28J60_H

#include "IO_Network.h"
#include "DIAG.h"
#include "EtherCard.h"

// The ethernet buffer is global to different protocol layers, to avoid almost
// all copying of data.
byte Ethernet::buffer[75]; // Need space for 32 byte payload and 42 byte header, at least.

class Net_ENC28J60 : public EtherCard {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _csPin;
  // Number of the current node (0-254)
  uint8_t _thisNode;
  // 6-byte MAC address.  Last byte will contain the node number (0-254).
  byte _address[6];
  // 4-byte IP address.  Last byte will contain the node number (0-254) or 255 for broadcast.
  byte _ipAddress[4];
  byte _gwAddress[4] = {192,168,1,254};
  byte _netMask[4] = {255,255,255,0};
  const uint16_t _port = 8900;
  uint8_t _packetBytesAvailable;

public:
  // Constructor performs static initialisation of the device object
  Net_ENC28J60 (int csPin) {
    _csPin = csPin;
    // Initialise with an arbitrary mac address.
    _address[0] = 0xEE;
    _address[1] = 0xCC;
    _address[2] = 0xEE;
    _address[3] = 0xEE;
    _address[4] = 0xCC;
    _address[5] = 0;

    _ipAddress[0] = 192;
    _ipAddress[1] = 168;
    _ipAddress[2] = 1;
    _ipAddress[3] = 0;

    _packetBytesAvailable = 0;
  }

  // Perform dynamic initialisation of the device
  bool begin(uint8_t thisNode) {
    _thisNode = thisNode;
    _address[5] = _thisNode;
    _ipAddress[3] = _thisNode;
    if (ether.begin(sizeof(Ethernet::buffer), _address, _csPin)) {
      ether.staticSetup(_ipAddress, _gwAddress, 0, _netMask);
      return true;
    } else {
      // Error in initialising
      DIAG(F("ENC28J60 Failed to initialise"));
      return false;
    }
  }

  // The following function should be called regularly to handle incoming packets.
  // Check if there is a received packet ready for reading.
  bool available() {
    uint16_t packetLen = ether.packetReceive();
      
    if (packetLen > 0) {
      // Packet received.  Check if it's one of ours.
      byte *gbp = Ethernet::buffer;
      uint8_t proto = gbp[IP_PROTO_P];
      uint16_t destPort = (gbp[UDP_DST_PORT_H_P] << 8) + gbp[UDP_DST_PORT_L_P];
      if (packetLen >= UDP_DATA_P && proto==IP_PROTO_UDP_V && destPort == _port) {
        // Yes, so mark that data is to be processed.
        _packetBytesAvailable = packetLen;
      } else {
        ether.packetLoop(packetLen);  // Some other protocol, so handle it.
        _packetBytesAvailable = 0;
      }
      return true;
    }
    return false;
  }

  // Read packet from the ethernet buffer, and return the number of bytes
  // that were read.
  uint8_t read(uint8_t buffer[], uint8_t size) {
    if (_packetBytesAvailable > 0) {
      // Mark that no packet to process
      _packetBytesAvailable = 0;

      // Check if there's room for the data
      uint8_t payloadSize = _packetBytesAvailable - UDP_DATA_P;
      if (size < payloadSize) return 0;

      memcpy(buffer, Ethernet::buffer+UDP_DATA_P, payloadSize);
      return payloadSize;
    }
    return 0;
  }

  // Wrapper functions for ENC28J60 sendUdp function.
  // The node parameter is either 0-254 (for specific nodes) or 255 (for broadcast).
  // This aligns with the subnet broadcast IP address of "x.x.x.255".
  bool sendCommand(uint8_t node, const uint8_t buffer[], uint8_t len) {
    _ipAddress[3] = node;
    ether.sendUdp((const char *)buffer, len, _port, _ipAddress, _port);
    return true;
  }

  void loop() { }

};

#endif //NET_ENC28J60_H