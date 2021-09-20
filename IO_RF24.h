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
 * Each node on the network is configured with a node number in the range 0-254.
 * The remoting configuration defines, for each pin to be available remotely,
 * the node number and the VPIN number on that node. The configuration must
 * match in all nodes, since it is used by the sending node to identify the node
 * and VPIN to which a write command is to be sent, and the VPIN number for a
 * sensor/input, and on the receiving node to identify the node from which a
 * sensor/input value is being sourced.
 *
 * The node number is also used as the first byte of the RF24's 5-byte address
 * field. Number 255 is treated as a multicast address.  All stations listen on
 * their own address and on the multicast address.
 *
 * All nodes send regular multicast packets containing the latest values of the
 * sensors as they know them.  On receipt of such a packet, each node extracts
 * the states of the sensors which are sourced by the originating node, and
 * updates the values in its own local data.  Thus, each node has a copy of the
 * states of all digital input pin values that are defined in the remoting
 * configuration.  Multicasts are sent frequently, so if one is missed
 * then, like a London bus, another will be along shortly.
 *
 * Commands (originating from write() or writeAnalogue() calls) are sent
 * immediately, directly from the originating node to the target node.  This
 * is done with acknowlegements enabled to maximise the probability of
 * successful delivery.
 *
 * The RF24 device receives and acknowledges data packets autonomously.
 * Therefore, this driver just needs to detect when a packet is received and
 * read and process its contents.  The time to read the packet is under 200us
 * typically.
 *
 * The RF24 is also capable of autonomously sending packets, processing
 * acknowledgements, and generating retries.  The driver writes the packet to
 * the device and then waits for notification of completion (success, or retries
 * exceeded) through the device's registers.  Similarly, the time to write a
 * packet is under 200us and, if we don't wait for the completion, we can allow
 * the processor to do other things while the transmission is in progress.
 * A write with ack can complete in under 600us, plus the time of turning the
 * receiver off and on.
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
      _radio.enableDynamicPayloads();  // variable length packets
      _radio.setAutoAck(true);
      _radio.enableDynamicAck(); // required for multicast to work
      _radio.setRetries(1, 5);  // Retry time=1*250+250us=500us, count=5.

      // Send and receive on address 255
      _address[0] = 255;
      _radio.openWritingPipe(_address);
      // Set to listen on the address 255
      _address[0] = 255;
      _radio.openReadingPipe(1, _address);
      // Also allow receives on own node address
      _address[0] = _thisNode;
      _radio.openReadingPipe(2, _address);
      _radio.startListening();

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

    // Check for incoming data (including ack payloads)
    if (_radio.available(NULL))
      processReceivedData();

    // Send out data update broadcasts once every 100ms
    if (currentMicros - _lastExecutionTime > (100 * + _thisNode % 20) * 1000UL ) {
      // Broadcast updates to all other nodes
      sendSensorUpdates();
      _lastExecutionTime = currentMicros;
    }
  }

  void _display() override {
    DIAG(F("nRF24L01 Configured on Vpin:%d-%d CEPin:%d CSNPin:%d"),
      _firstVpin, _firstVpin+_nPins-1, _cePin, _csnPin);
  }

private:
  // TODO: Reduce number of regular transmissions.  For example, when one of the values sourced
  // on this node changes, then send a defined number of transmissions (e.g. 3), but otherwise
  // send them once a second or so.

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
      // Set up to broadcast updates
      sendCommand(255, outBuffer, byteIndex);
      //DIAG(F("Sent %d bytes"), byteIndex);
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
            // Process incoming value if it's come from the pin source node
            uint8_t pinSource = _pinDefs[currentPin].node;
            if (sendingNode == pinSource) {
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

  // Wrapper functions for RF24 send functions.  If node=255, then
  //  the packet is to be sent as a multicast without acknowledgements.
  //  The multicast message takes ~400us. A further 260us is required to turn
  //  the receiver off and on for the transmission, totalling 660us.
  //  If the node is not 255, then the packet will be sent directly to the
  //  addressed node, with acknowledgement requested.  If no acknowledgement is
  //  received, then the device will retry up to the defined maximum number of
  //  retries.  This will take longer than a multicast.  For example, with
  //  setRetries(1,3) the timeout is 500us and a maximum of 3 retries are
  //  carried out, so the operation will take as much as 2.26 milliseconds if
  //  the node in question is not responding, and as little as 890us if the 
  //  ack is received immediately (including turning receiver on/off).
  //
  // TODO: Separate writeFast from txStandBy to avoid blocking during transmission
  //  and during retries.
  //
  bool sendCommand(uint8_t node, uint8_t *buffer, uint8_t len) {
    _address[0] = node;
    _radio.openWritingPipe(_address);
    _radio.stopListening();
    // Multicast if destination node is 255
    bool ok = _radio.writeFast(buffer, len, (node==255)); 
    _radio.txStandBy();
    _radio.startListening();
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