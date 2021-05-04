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

#ifndef I2CMANAGER_AVR_H
#define I2CMANAGER_AVR_H

#include <Arduino.h>
#include "I2CManager.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master staus codes                      
#define TWI_START                  0x08  // START has been transmitted  
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter staus codes                      
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

// TWI Master Receiver staus codes  
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#define TWI_TWBR  ((F_CPU / I2C_FREQ) - 16) / 2 // TWI Bit rate Register setting.

/***************************************************************************
 *  Set I2C clock speed register.
 ***************************************************************************/
void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  TWBR = ((F_CPU / i2cClockSpeed) - 16) / 2;
}

/***************************************************************************
 *  Initialise I2C registers.
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{
  TWSR = 0;
  TWBR = TWI_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (1<<TWINT);                                // Clear interrupt flag
 
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_startTransaction(I2CRB *)
{
  TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTA);  // Send Start
}

/***************************************************************************
 *  Test if I2C is in stopped state
 ***************************************************************************/
bool I2CManagerClass::I2C_isStopped(I2CRB *)
{
  return !( TWCR & _BV(TWSTO) );
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close(I2CRB *)
{
  // disable TWI
  TWDR = 0xFF;                       // Default content = SDA released.
  TWCR = (1<<TWINT)|(1<<TWSTO);      // send stop and clear interrupt flag
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler.
 ***************************************************************************/
void I2CManagerClass::I2C_handle(I2CRB * t) 
{
  
  if( (status == I2C_STATUS_FREE) || (status == I2C_STATUS_CLOSING ) ) {
    TWCR |= (1<<TWINT);  // Clear interrupt
    return;
  }
  
  uint8_t twsr = TWSR & 0xF8;

  switch (twsr) {
    case TWI_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
      if (txCount < t->writeLen)
      {
        if (t->operation == OPERATION_SEND_P)
          TWDR = GETFLASH(t->writeBuffer + (txCount++));
        else
          TWDR = t->writeBuffer[txCount++];
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
      }else
      {
        TWDR = 0xFF;
        if( t->operation == OPERATION_SEND || t->operation == OPERATION_SEND_P)
        {
          // Initiate a STOP condition.
          TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTO);
          status = I2C_STATUS_OK;
        }
        else // request data (repeated start)
        {
           t->operation = OPERATION_REQUEST_READ;
           // repeated start
           TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA);                             //
        }
      }
      break;
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted
      t->readBuffer[rxCount++] = TWDR;
      /* fallthrough */
    case TWI_MRX_ADR_ACK:       // SLA+R has been tramsmitted and ACK received

      if (rxCount+1 < t->readLen )              // Detect the last byte to NACK it.
      {
        // send ack
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
      }else                    // Send NACK after next reception
      {
        // send nack
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);                                 // 
      }    
      break;
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
      t->readBuffer[rxCount++] = TWDR;

      // send stop
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTO);
      status = I2C_STATUS_OK;
      break;
    case TWI_START:             // START has been transmitted  
    case TWI_REP_START:         // Repeated START has been transmitted
      txCount = 0;              // Set buffer pointer to 0
      
      switch( t->operation ) {
        case OPERATION_READ:
        case OPERATION_REQUEST_READ:
          TWDR = (t->i2cAddress << 1) | 1;
          break;
        case OPERATION_REQUEST:
        case OPERATION_SEND:
        case OPERATION_SEND_P:
          TWDR = (t->i2cAddress << 1) | 0;
          break;
      }
      TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
      break;
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTA);                                // Initiate (re)start
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been transmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been transmitted and NACK received    
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTO);
      status = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      break;
    case TWI_MTX_DATA_NACK:     // Data byte has been transmitted and NACK received
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTO);
      status = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      break;
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA)|(1<<TWSTO);    // send stop
      status = I2C_STATUS_TRANSMIT_ERROR;
  }
}

ISR(TWI_vect) {
  I2CManager.handleInterrupt();
}

#endif