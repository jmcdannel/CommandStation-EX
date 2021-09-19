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
 * RF24 Mode:
 *   Channel: 76
 *   Bit rate: 2MHz
 *
 * One station (normally the CS) is configured as Master (node 0), and
 * the others as slaves (nodes 1-255).
 *
 * The master sends regular updates to the slaves, one at a time, containing the
 * latest values of the sensors distributed around the network. Slave nodes send
 * their regular updates to the master as 'ack payloads', i.e. in response to
 * receiving a message from the master. This strategy should reduce collisions
 * (where two nodes attempt to transmit at the same time, rendering both
 * messages unintelligible). It also minimises the time taken in the slave
 * microcontroller to write messages.  Once the RF24 has been loaded with the
 * 'ack payload', it autonomously takes care of receiving the master message,
 * changing into transmit mode, sending the 'ack' and its payload, and changing
 * back into receive mode, with no intervention from the microcontroller.
 * When the operation has completed, then the received message can be read from
 * the RF24's input buffers.
 *
 * Commands (originating from write() or writeAnalogue() calls) are sent
 * immediately, directly from the originating node to the target node.
 *
 * The node number is part of the 5-byte address used for sending/receiving
 * data.  A node listens on its own address for messages sent directly from the
 * master or from another slave.  When a packet is sent it is addressed with the
 * node number of the target node.  Thus, up to 255 slave devices may be
 * addressed in addition to the master device.
 *
 * The time taken to send or to receive a message in the Arduino has been
 * measured and is typically below 100us, except where the output buffers are
 * full.  In this case the program has to wait for an earlier packet to be
 * transmitted and acknowledged before the current packet can be written to the
 * device.  The regular message traffic is spaced such that this will rarely
 * happen.
 * 
 */

#ifndef IO_RF24_H
#define IO_RF24_H

#include "IODevice.h"
#include "RF24.h"

typedef struct { uint8_t node; VPIN vpin; } RPIN;

class RF24Net : public IODevice {

private:
  // pins must be arduino GPIO pins, not extender pins or HAL pins.
  int _cePin = -1;
  int _csnPin = -1;
  const RPIN *_pinDefs;
  // Time of last loop execution
  unsigned long _lastExecutionTime;
  // Current digital values for remoted pins
  int *_pinValues;
  // Number of the current node (0=master, 1-255=slave)
  uint8_t _thisNode;
  // Maximum slave node number to be polled.  Set this lower to avoid unnecessary poll cycles.
  const uint8_t _maxNode = 5;
  // 5-byte nRF24L01 address.  First byte will contain the node number
  byte _address[5] = {0x00, 0xCC, 0xEE, 0xEE, 0xCC};
  // Maximum size of payload (must be 32 or less)
  static const uint8_t maxPayloadSize = 32;
  // Current node being sent sensor data and polled
  uint8_t _currentSendNode = 0;
  
  RF24 _radio;

  // List of network commands
  enum : uint8_t {
    NET_CMD_WRITE,
    NET_CMD_WRITEANALOGUE,
    NET_CMD_VALUEUPDATE,
  };

public:
  // Constructor performs static initialisation of the device object
  RF24Net (VPIN firstVpin, int nPins, uint8_t thisNode, const RPIN pinDefs[], int cePin, int csnPin) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _cePin = cePin;
    _csnPin = csnPin;
    _thisNode = thisNode;
    _pinDefs = pinDefs; 
    _address[0] = 0x00;
    _address[1] = 0xCC;
    _address[2] = 0xEE;
    _address[3] = 0xEE;
    _address[4] = 0xCC;
    _pinValues = (int *)calloc(nPins, sizeof(int));  // Allocate space for input values.
    addDevice(this);
  }

  // Static create function provides alternative way to create object
  static void create(VPIN firstVpin, int nPins, uint8_t thisNode, const RPIN pinDefs[], int cePin, int csnPin) {
    new RF24Net(firstVpin, nPins, thisNode, pinDefs, cePin, csnPin);
  }

protected:
  // _begin function called to perform dynamic initialisation of the device
  void _begin() override {
#if defined(DIAG_IO)
    _display();
#endif
    if (_radio.begin(_cePin, _csnPin)) {
      // Device initialisation OK, set up parameters
      _radio.setDataRate(RF24_2MBPS);
      _radio.setPALevel(RF24_PA_LOW);
      _radio.setChannel(76);
      _radio.setAutoAck(true);
      _radio.enableAckPayload();
      _radio.enableDynamicAck();
      _radio.enableDynamicPayloads();
      _radio.setRetries(1, 1);  // Retry time=1*250+250us=500us, count=1.

      _radio.openWritingPipe(_address);
      // Set to listen on the address coresponding to this node number
      _address[0] = _thisNode;
      _radio.openReadingPipe(1, _address);
      // if (_thisNode != 0) {
      //   // Set also to listen for messages sent to master
      //   _address[0] = 0;
      //   _radio.openReadingPipe(2, _address);
      // }

      _display();
      _deviceState = DEVSTATE_NORMAL;
    } else {
      // Error in initialising
      DIAG(F("nRF24L01 Failed to initialise"));
      _deviceState = DEVSTATE_FAILED;
    }
    _lastExecutionTime = micros();
  }

  // _read function - just return _value (updated in _loop when message received from remote node)
  int _read(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    return _pinValues[pin];
  }

  // _write (digital) - send command directly to the appropriate remote node.
  void _write(VPIN vpin, int value) override {
    // Send message
    int pin = vpin - _firstVpin;
    uint8_t node = _pinDefs[pin].node;
    VPIN remoteVpin = _pinDefs[pin].vpin;
    if (node != _thisNode) {
      #ifdef DIAG_IO
      DIAG(F("RF24: write(%d,%d)=>send(%d,\"write(%d,%d)\")"), vpin, value, node, remoteVpin, value);
      #endif

      outBuffer[0] = node;
      outBuffer[1] = NET_CMD_WRITE;
      outBuffer[2] = getMsb(remoteVpin);
      outBuffer[3] = getLsb(remoteVpin);
      outBuffer[4] = (uint8_t)value;
      // Set up to send to the specified node address
      sendCommand(node, outBuffer, 5);
    }
  }

  // _writeAnalogue - send command directly to the appropriate remote node.
  void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override {
    // Send message
    int pin = vpin - _firstVpin;
    uint8_t node = _pinDefs[pin].node;
    VPIN remoteVpin = _pinDefs[pin].vpin;
    if (node != _thisNode) {
      #ifdef DIAG_IO
      DIAG(F("RF24: writeAnalogue(%d,%d,%d,%d)=>send(%d,\"writeAnalogue(%d,%d,...)\")"), 
        vpin, value, param1, param2, node, remoteVpin, value);
      #endif

      outBuffer[0] = node;
      outBuffer[1] = NET_CMD_WRITEANALOGUE;
      outBuffer[2] = getMsb(remoteVpin);
      outBuffer[3] = getLsb(remoteVpin);
      outBuffer[4] = getMsb(value);
      outBuffer[5] = getLsb(value);
      outBuffer[6] = param1;
      outBuffer[7] = getMsb(param2);
      outBuffer[8] = getLsb(param2);
      // Set up to send to the specified node address
      sendCommand(node, outBuffer, 9);
    }
  }

  // _loop function - check for, and process, received data from RF24, and send any
  // updates that are due.
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    // Send out data updates once every 5ms
    if (currentMicros - _lastExecutionTime > 5 * 1000UL) {
      if (_thisNode == 0) {
        // Master - Send sensor updates to next slave.
        _currentSendNode++;
        if (_currentSendNode > _maxNode) 
          _currentSendNode = 1;
        if (_currentSendNode <= _maxNode) 
          sendSensorUpdates();
      } else {
        // Slave, collect sensor updates ready for sending
        sendSensorUpdates();
      }
      _lastExecutionTime = currentMicros;

    }

    // Check for incoming data (including ack payloads)
    if (_radio.available(NULL))
      processReceivedData();

  }

  void _display() override {
    DIAG(F("nRF24L01 Configured on Vpin:%d-%d CEPin:%d CSNPin:%d"),
      _firstVpin, _firstVpin+_nPins-1, _cePin, _csnPin);
  }

private:
  void sendSensorUpdates() { 
    // On master and on slave, send pin states to other nodes
    outBuffer[0] = _thisNode;  // Originating node
    outBuffer[1] = NET_CMD_VALUEUPDATE;
    // TODO: Handle more than 8*28=224 sensors! For this, we will need to start a new packet
    // when the first one is full.
    uint16_t pinCount = min(_nPins, (maxPayloadSize-4)*8);
    VPIN remoteVpin = _firstVpin;
    outBuffer[2] = getMsb(remoteVpin);
    outBuffer[3] = getLsb(remoteVpin);
    uint8_t byteIndex = 4;
    uint8_t byteValue = 0;
    uint8_t mask = 1;
    // Send current value if local pin, or stored value if remote pin
    for (uint16_t pin=0; pin<pinCount; pin++) {
      if (_pinDefs[pin].node == _thisNode) {
        // Local pin, read and send current state of input
        VPIN localVpin = _pinDefs[pin].vpin;
        bool state = IODevice::read(localVpin);
        // Store state in remote values array
        _pinValues[pin] = state;
      }
      // Store from pinValues array in buffer
      if (_pinValues[pin]) byteValue |= mask;
      mask <<= 1;
      if (mask == 0) { // Byte completed?
        // Store byte value in buffer and reset for next byte.
        outBuffer[byteIndex++] = byteValue;
        byteValue = 0;
        mask = 0;
      }
    }
    if (mask > 1) {
      // Byte started but not finished, so store that too.
      outBuffer[byteIndex++] = byteValue;
    }

    if (_thisNode == 0) {  // Master
      // Set up to send to the node currently being addressed, and to receive
      // the acknowledgements.
      sendCommand(_currentSendNode, outBuffer, byteIndex);
      //DIAG(F("Sent %d bytes to node %d"), byteIndex, _currentSendNode);
    } else { // Slave
      // Put in queue as ack payload, after first flushing any existing payload
      // that hasn't been sent.
      //_radio.flush_tx();
      _radio.writeAckPayload(1, outBuffer, byteIndex);
      //DIAG(F("Queued %d bytes"), byteIndex);
    }
  }

  // Read next packet from the device's input buffers.  Decode the message, 
  // and take the appropriate action.
  // The packet may be a command to do an output write (digital or analogue), or
  // it may be an update for digital input signals.
  // For digital input signals, the values are propagated from the node that is
  // the pin source, via the master, to all the other nodes.  
  void processReceivedData() {
    // Read received data from input pipe
    byte size = _radio.getDynamicPayloadSize();
    // if (size > maxPayloadSize) return; // Packet too long to read!!
    // Read packet
    _radio.read(inBuffer, size);
    // Extract command type from packet.
    uint8_t command = inBuffer[1];
    // Process received data 
    switch (command) {
      case NET_CMD_WRITE: // Digital write command
        {
          uint8_t targetNode = inBuffer[0];
          if (targetNode == _thisNode) {
            VPIN vpin = makeWord(inBuffer[2], inBuffer[3]);
            uint8_t state = inBuffer[4];
            IODevice::write(vpin, state); 
          } else {
            sendCommand(targetNode, inBuffer, size);
          }
        }
        break;
      case NET_CMD_WRITEANALOGUE:  // Analogue write command
        {
          uint8_t targetNode = inBuffer[0];
          if (targetNode == _thisNode) {
            VPIN vpin = makeWord(inBuffer[2], inBuffer[3]);
            int value = makeWord(inBuffer[4], inBuffer[5]);
            uint8_t param1 = inBuffer[6];
            uint16_t param2 = makeWord(inBuffer[7], inBuffer[8]);
            IODevice::writeAnalogue(vpin, value, param1, param2);
            // Set the local value for the pin, used by isBusy(),
            // and subsequently updated by the remote node.
            _pinValues[vpin-_firstVpin] = true;
          } else {
            sendCommand(targetNode, inBuffer, size);
          }
        }
        break;
      case NET_CMD_VALUEUPDATE: // Updates of input states (sensors etc).
        {
          uint8_t sendingNode = inBuffer[0];
          //DIAG(F("Node %d Rx %x"), sendingNode, inBuffer[4]);
          VPIN vpin = makeWord(inBuffer[2], inBuffer[3]);
          int currentPin = vpin-_firstVpin;
          for (uint16_t bitIndex = 4*8; (bitIndex < size*8) && (currentPin < _nPins); bitIndex++) {
            // Process incoming value if it's come to the master from the pin source node,
            // or if it's come from the master and this is a slave.
            uint8_t pinSource = _pinDefs[currentPin].node;
            if ((sendingNode = pinSource && _thisNode == 0) || (sendingNode == 0 && _thisNode != 0)) {
              uint8_t bitMask = 1 << (bitIndex % 8);
              uint8_t byteIndex = bitIndex / 8;
              _pinValues[currentPin] = (inBuffer[byteIndex] & bitMask) ? 1 : 0;
              //if (pinNode == _thisNode) { // Local pin }
            }
            currentPin++;
          }
        }
        break;
      default:
        break;
    }
  }


  // Wrapper functions for RF24 send functions
  bool sendCommand(uint8_t node, uint8_t *buffer, uint8_t len) {
    _address[0] = node;
    _radio.openWritingPipe(_address);
    bool ok = _radio.writeBlocking(buffer, len, 2); // 2ms timeout
    return ok;
  }

  // Helper functions for packing/unpacking buffers.
  inline uint16_t makeWord(uint8_t msb, uint8_t lsb) {
    return ((uint16_t)msb << 8) | lsb;
  }
  inline uint8_t getMsb(uint16_t w) {
    return w >> 8;
  }
  inline uint8_t getLsb(uint16_t w) {
    return w & 0xff;
  }
  // Data space for actual input and output buffers.  
  uint8_t inBuffer[maxPayloadSize];
  uint8_t outBuffer[maxPayloadSize];

};

#endif //IO_RF24Net4_H