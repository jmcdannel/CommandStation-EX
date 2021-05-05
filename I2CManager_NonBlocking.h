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

#ifndef I2CMANAGER_NONBLOCKING_H
#define I2CMANAGER_NONBLOCKING_H

#include <Arduino.h>
#include "I2CManager.h"

// This module is only compiled if USE_WIRE is not defined, so undefine it here
// to get intellisense to work correctly.
#undef USE_WIRE


#define DISABLED_IRQ_SECTION_START {                           \
                                     uint8_t sreg = SREG;      \
                                     cli();

#define DISABLED_IRQ_SECTION_LEAVE   SREG = sreg;              \
                                   }

typedef enum : uint8_t
{
  I2C_STATUS_CLOSING=254,
  I2C_STATUS_FREE=255,
} InternalStatusEnum;

/***************************************************************************
 * Initialise the I2CManagerAsync class.
 ***************************************************************************/
void I2CManagerClass::_initialise()
{
  queueHead = queueTail = NULL;
  status = I2C_STATUS_FREE;
  I2C_init();
}

/***************************************************************************
 * Helper function to start operations, if the I2C interface is free and
 * there is a queued request to be processed.
 * MUST BE CALLED WITH INTERRUPTS INHIBITED.
 ***************************************************************************/
void I2CManagerClass::startTransaction() {
  if( queueHead && (status == I2C_STATUS_FREE) ) {
    status = I2C_STATUS_PENDING;
    rxCount = txCount = 0;
    I2C_startTransaction( queueHead );
    startTime = micros();
  }
}


/***************************************************************************
 *  Function to queue a request block and initiate operations.
 ***************************************************************************/
void I2CManagerClass::queueRequest(I2CRB *req) {
  req->status = I2C_STATUS_PENDING;
  req->nextRequest = NULL;

  DISABLED_IRQ_SECTION_START;
  if (!queueTail) 
    queueHead = queueTail = req;  // Only item on queue
  else
    queueTail = queueTail->nextRequest = req; // Add to end

  startTransaction();
  DISABLED_IRQ_SECTION_LEAVE;
}

/***************************************************************************
 *  Initiate a write to an I2C device (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write(uint8_t i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen, I2CRB *req) {
  // Make sure previous request has completed.
  req->wait();
  req->setWriteParams(i2cAddress, writeBuffer, writeLen);
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(uint8_t i2cAddress, const uint8_t * writeBuffer, uint8_t writeLen, I2CRB *req) {
  // Make sure previous request has completed.
  req->wait();
  req->setWriteParams(i2cAddress, writeBuffer, writeLen);
  req->operation = OPERATION_SEND_P;
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 *  Initiate a read from the I2C device, optionally preceded by a write 
 *   (non-blocking operation)
 ***************************************************************************/
uint8_t I2CManagerClass::read(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen, 
    const uint8_t *writeBuffer, uint8_t writeLen, I2CRB *req)
{
  // Make sure previous request has completed.
  req->wait();
  req->setRequestParams(i2cAddress, readBuffer, readLen, writeBuffer, writeLen);
  queueRequest(req);
  return I2C_STATUS_OK;
}

/***************************************************************************
 * checkForTimeout() function, called from isBusy() and wait() to cancel
 * requests that are taking too long to complete.
 ***************************************************************************/
void I2CManagerClass::checkForTimeout() {
  unsigned long currentMicros = micros();
  DISABLED_IRQ_SECTION_START;
  volatile I2CRB *t = queueHead;
  if (t && timeout > 0) {
    // Check for timeout
    if (currentMicros - startTime > timeout) { 
      // Excessive time. Dequeue request
      queueHead = t->nextRequest;
      if (!queueHead) queueTail = NULL;
      // Post request as timed out.
      t->status = I2C_STATUS_TIMEOUT;
      // Reset TWI interface so it is able to continue
      // Try close and init, not entirely satisfactory but sort of works...
      I2C_close(NULL);  // Shutdown and restart twi interface
      I2C_init();
      status = I2C_STATUS_FREE;
      // Initiate next queued request
      startTransaction();
    }
  }
  DISABLED_IRQ_SECTION_LEAVE;
}

/***************************************************************************
 * Interupt handler.  Call I2C state machine, and dequeue request
 * if completed.
 ***************************************************************************/
void I2CManagerClass::handleInterrupt() {
  I2CRB * t = queueHead;
  if (!t) return;  // No active operation
  I2C_handle(t);

  if (status!=I2C_STATUS_PENDING) {
    // Remove completed request from queue
    queueHead = t->nextRequest;
    if (!queueHead) queueTail = queueHead;

    // Update request status
    if (t->operation == OPERATION_REQUEST_READ) 
      t->operation = OPERATION_REQUEST; // reinstate original operation field
    t->nBytes = rxCount;
    t->status = status;

    status = I2C_STATUS_FREE;

    // Start next request (if any)
    startTransaction();
  }
}

#endif