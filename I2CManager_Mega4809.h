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

#ifndef I2CMANAGER_MEGA4809_H
#define I2CMANAGER_MEGA4809_H

#include <Arduino.h>
#include "I2CManager.h"

//#define TWI_TWBR  ((F_CPU / I2C_FREQ) - 16) / 2 // TWI Bit rate Register setting.

/***************************************************************************
 *  Set I2C clock speed register.
 ***************************************************************************/
void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  // TODO: set clock speed register
}

/***************************************************************************
 *  Initialise I2C registers.
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{ 
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  PORTMUX.TWISPIROUTEA |= TWI_MUX;

  TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm;
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;

  // TODO: set clock speed register
}

/***************************************************************************
 *  Initiate a start bit for transmission, followed by address and R/W
 ***************************************************************************/
void I2CManagerClass::I2C_startTransaction() {
  // If anything to send, send it first.
  if (operation == OPERATION_READ || operation == OPERATION_REQUEST)
    TWI0.MADDR = (currentRequest->i2cAddress << 1) | 0;
  else
    TWI0.MADDR = (currentRequest->i2cAddress << 1) | 1;
}

/***************************************************************************
 *  Initiate a stop bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_stopTransaction() {
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close() {
  I2C_stopTransaction();
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler.
 ***************************************************************************/
void I2CManagerClass::I2C_handleInterrupt() {
  
  if( (status == I2C_STATUS_FREE) || (status == I2C_STATUS_CLOSING ) ) 
    return;
    
  // Find current (active) request block.
  I2CRB *t = queueHead;
  
  uint8_t currentStatus = TWI0.MSTATUS;

  if (currentStatus & TWI_ARBLOST_bm) {
    status = I2C_STATUS_ARBITRATION_LOST;
    TWI0.MSTATUS = currentStatus; // clear all flags
  } else if (currentStatus & TWI_BUSERR_bm) {
    status = I2C_STATUS_BUS_ERROR;
    TWI0.MSTATUS = currentStatus; // clear all flags
  } else if (currentStatus & TWI_WIF_bm) {
    // Master write completed
    if (currentStatus & TWI_RXACK_bm) {
      // Acked
      if (bytesToSend) {
        // Send next byte
        if (t->operation == OPERATION_SEND_P)
          TWI0.MDATA = GETFLASH(t->writeBuffer + (txCount++));
        else
          TWI0.MDATA = t->writeBuffer[txCount++];
        bytesToSend--;
      } else if (bytesToReceive) {
          // Send repeated start, address and read bit.
          TWI0.MADDR = (t->i2cAddress << 1) | 1;
      } else {
        // No more data to send/receive. Initiate a STOP condition.
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        status = I2C_STATUS_OK;  // Done
      }
    } else {
      // Nacked, send stop.
      TWI0.MCTRLB = TWI_MCMD_STOP_gc;
      status = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
    }
  } else if (currentStatus & TWI_RIF_bm) {
    // Master read completed.
    if (bytesToReceive) {
      t->readBuffer[rxCount++] = TWI0.MDATA;  // Store received byte
      bytesToReceive--;
    } else { 
      // Buffer full, issue nack/stop
      TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
      status = I2C_STATUS_OK;
    }
  } else {
    // Unexpected state, finish.
    status = I2C_STATUS_UNEXPECTED_ERROR;
  }
}



ISR(TWI0_TWIM_vect) {
  I2CManagerClass::handleInterrupt();
}

#endif