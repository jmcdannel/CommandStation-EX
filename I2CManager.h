/*
 *  © 2021, Neil McKechnie. All rights reserved.
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

#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include "FSH.h"

/* 
 * Manager for I2C communications.  For portability, it allows use 
 * of the Wire class, but also has a native implementation for AVR
 * which supports non-blocking queued I/O requests.
 * 
 * Helps to avoid calling Wire.begin() multiple times (which is not)
 * entirely benign as it reinitialises).
 * 
 * Also helps to avoid the Wire clock from being set, by another device
 * driver, to a speed which is higher than a device supports.
 * 
 * Thirdly, it provides a convenient way to check whether there is a 
 * device on a particular I2C address.
 * 
 * Non-blocking requests are issued by creating an I2C Request Block
 * (I2CRB) which is then added to the I2C manager's queue.  The 
 * application refers to this block to check for completion of the
 * operation, and for reading completion status.
 * 
 * Examples:
 *  I2CRB rb;
 *  uint8_t status = I2CManager.write(address, buffer, sizeof(buffer), &rb);
 *  ...
 *  if (!rb.isBusy()) {
 *    status = rb.status;
 *    // Repeat write
 *    I2CManager.queueRequest(&rb);
 *    ...
 *    status = rb.wait(); // Wait for completion and read status
 *  }
 *  ...
 *  I2CRB rb2;
 *  outbuffer[0] = 12;  // Register number in I2C device to be read
 *  rb2.setRequestParams(address, inBuffer, 1, outBuffer, 1);
 *  status = I2CManager.queueRequest(&rb2);
 *  if (status == I2C_STATUS_OK) { 
 *    status = rb2.wait();
 *    if (status == I2C_STATUS_OK) {
 *      registerValue = inBuffer[0];
 *    }
 *  }
 *  ...
 *  
 * Synchronous (blocking) calls are also possible, e.g. 
 *  status = I2CManager.write(address, buffer, sizeof(buffer));
 * 
 * When using non-blocking requests, neither the I2CRB nor the input or output
 * buffers should be modified until the I2CRB is complete (not busy).
 * 
 * Timeout monitoring is possible, but requires that the following call is made
 * reasonably frequently in the program's loop() function:
 *  I2CManager.loop();
 * 
 */

// Define USE_WIRE to force use of the Wire/TWI library (blocking I2C).
//#define USE_WIRE

enum {
  I2C_STATUS_OK=0,
  I2C_STATUS_TRUNCATED=1,
  I2C_STATUS_DEVICE_NOT_PRESENT=2,
  I2C_STATUS_TRANSMIT_ERROR=3,
  I2C_STATUS_NEGATIVE_ACKNOWLEDGE=4,
  I2C_STATUS_TIMEOUT=5,
  I2C_STATUS_PENDING=253,
};

typedef enum : uint8_t
{
  OPERATION_READ = 1,
  OPERATION_REQUEST = 2,
  OPERATION_SEND = 3,
  OPERATION_SEND_P = 4,
  OPERATION_REQUEST_READ = 5, // Used internally as second phase of REQUEST
} OperationEnum;


// Default I2C frequency
#ifndef I2C_FREQ
#define I2C_FREQ    400000
#endif

// Struct defining a request context for an I2C operation.
struct I2CRB {
  volatile uint8_t status; // Completion status, or pending flag (updated from IRC)
  volatile uint8_t nBytes; // Number of bytes read (updated from IRC)

  uint8_t wait();
  bool isBusy();
  inline void init() { status = I2C_STATUS_OK; };
  void setReadParams(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen);
  void setRequestParams(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen, const uint8_t *writeBuffer, uint8_t writeLen);
  void setWriteParams(uint8_t i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen);

  uint8_t writeLen;
  uint8_t readLen;
  uint8_t operation;
  uint8_t i2cAddress;
  uint8_t *readBuffer;
  const uint8_t *writeBuffer;
#ifndef USE_WIRE
  I2CRB *nextRequest;
#endif
};

// I2C Manager
class I2CManagerClass {
public:

  // If not already initialised, initialise I2C (wire).
  void begin(void);
  // Set clock speed to the lowest requested one.
  void setClock(uint32_t speed);
  // Force clock speed 
  void forceClock(uint32_t speed);
  // Check if specified I2C address is responding.
  uint8_t checkAddress(uint8_t address);
  inline bool exists(uint8_t address) {
    return checkAddress(address)==I2C_STATUS_OK;
  }
  // Write a complete transmission to I2C from an array in RAM
  uint8_t write(uint8_t address, const uint8_t buffer[], uint8_t size);
  uint8_t write(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb);
  // Write a complete transmission to I2C from an array in Flash
  uint8_t write_P(uint8_t address, const uint8_t buffer[], uint8_t size);
  uint8_t write_P(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb);
  // Write a transmission to I2C from a list of bytes.
  uint8_t write(uint8_t address, uint8_t nBytes, ...);
  // Write a command from an array in RAM and read response
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
    const uint8_t writeBuffer[]=NULL, uint8_t writeSize=0);
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
    const uint8_t writeBuffer[], uint8_t writeSize, I2CRB *rb);
  // Write a command from an arbitrary list of bytes and read response
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
    uint8_t writeSize, ...);
  void queueRequest(I2CRB *req);

  // Function to abort long-running operations.
  void checkForTimeout();

private:
  bool _beginCompleted = false;
  bool _clockSpeedFixed = false;
  uint32_t _clockSpeed = 400000L;  // 400kHz max on Arduino.
  int gpioExtenderInterruptPin = -1; // Arduino pin number for chained interrupts

  // Finish off request block by waiting for completion and posting status.
  uint8_t finishRB(I2CRB *rb, uint8_t status);

  void _setClock(unsigned long i2cClockSpeed);
  void _initialise();

#ifndef USE_WIRE
    // I2CRB structs are queued on the following two links.
    // If there are no requests, both are NULL.
    // If there is only one request, then queueHead and queueTail both point to it.
    // Otherwise, queueHead is the pointer to the first request in the queue and
    // queueTail is the pointer to the last request in the queue.
    // Within the queue, each request's nextRequest field points to the 
    // next request, or NULL.
    // Mark volatile as they are updated by IRC and read/written elsewhere.
    I2CRB * volatile queueHead;
    I2CRB * volatile queueTail;
    volatile uint8_t status;

    uint8_t txCount;
    uint8_t rxCount;
    unsigned long startTime;

    unsigned long timeout = 0; // Transaction timeout in microseconds.  0=disabled.
    
    void                    startTransaction();
    
    void                    I2C_init();
    void                    I2C_handle(I2CRB *);
    void                    I2C_startTransaction(I2CRB *);
    bool                    I2C_isStopped(I2CRB *);
    void                    I2C_close(I2CRB *);
    
  public:
    void setTimeout(unsigned long value) { timeout = value;};

    void handleInterrupt();
#endif


};

extern I2CManagerClass I2CManager;

#endif