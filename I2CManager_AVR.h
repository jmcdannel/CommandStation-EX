#include <Arduino.h>
#include "I2CManager.h"

#include <avr/io.h>
#include <avr/interrupt.h>

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

  tries = 0;
  retryCount = 0;

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
    tries = 0;
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
 * loop() function, called from program's loop to do timeouts of stalled 
 * transfers.
 ***************************************************************************/
void I2CManagerClass::loop() {
  unsigned long currentMicros = micros();
  DISABLED_IRQ_SECTION_START;
  struct I2CRB *t = queueHead;
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

// TWI Slave Transmitter staus codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = 0); ACK has been received

// TWI Slave Receiver staus codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

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
  TWCR = (0<<TWEN)|                                 // Disable TWI-interface and release TWI pins.
         (0<<TWIE)|(1<<TWINT)|                      // Disable Interrupt and clear interrupt flag
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
         (0<<TWWC);                                 //
 
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_startTransaction(I2CRB *)
{
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
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
  TWCR = (0<<TWEN)|
         (0<<TWIE)|(1<<TWINT)| // Clear interrupt flag
         (0<<TWEA)|(0<<TWSTA)|
         (0<<TWSTO)|(0<<TWWC);
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler.
 ***************************************************************************/
void I2CManagerClass::I2C_handle(I2CRB * t) 
{
//  if( ! ( TWCR & (1<<TWINT) ) )
//    return;
  
  if( (status == I2C_STATUS_FREE) || (status == I2C_STATUS_CLOSING ) ) {
    TWCR |= (1<<TWINT);
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
        TWCR = (1<<TWEN)|
               (1<<TWIE)|(1<<TWINT)|
               (1<<TWEA)|(0<<TWSTA)|
               (0<<TWSTO)|(0<<TWWC);
      }else
      {
        TWDR = 0xFF;
        if( t->operation == OPERATION_SEND || t->operation == OPERATION_SEND_P)
        {
          // Initiate a STOP condition.
          TWCR = (1<<TWEN)|
          (0<<TWIE)|(1<<TWINT)|   // disable and clear interrupt
          (1<<TWEA)|(0<<TWSTA)|
          (1<<TWSTO)|(0<<TWWC);

          status = I2C_STATUS_OK;
        }
        else // request data (repeated start)
        {
           t->operation = OPERATION_REQUEST_READ;
           // repeated start
           TWCR = (1<<TWEN)|
                  (1<<TWIE)|(1<<TWINT)|
                  (0<<TWEA)|(1<<TWSTA)|
                  (0<<TWSTO)|(0<<TWWC);                             //
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
        TWCR = (1<<TWEN)|
               (1<<TWIE)|(1<<TWINT)|
               (1<<TWEA)|(0<<TWSTA)|
               (0<<TWSTO)|(0<<TWWC);
      }else                    // Send NACK after next reception
      {
        // send nack
        TWCR = (1<<TWEN)|
               (1<<TWIE)|(1<<TWINT)|
               (0<<TWEA)|(0<<TWSTA)|
               (0<<TWSTO)|(0<<TWWC);                                 // 
      }    
      break;
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
      t->readBuffer[rxCount++] = TWDR;
      
      // send stop
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|   // disable and clear interrupt
             (1<<TWEA)|(0<<TWSTA)|
             (1<<TWSTO)|(0<<TWWC);

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
      TWCR = (1<<TWEN)|
             (1<<TWIE)|(1<<TWINT)|
             (1<<TWEA)|(0<<TWSTA)|
             (0<<TWSTO)|(0<<TWWC);
      break;
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag
             (1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been transmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been transmitted and NACK received    
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|   // disable and clear interrupt
             (1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|
             (0<<TWWC);
      status = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      break;
    case TWI_MTX_DATA_NACK:     // Data byte has been transmitted and NACK received
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|   // disable and clear interrupt
             (1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|
             (0<<TWWC);
      status = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      break;
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|   // disable and clear interrupt
             (1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|
             (0<<TWWC);
      status = I2C_STATUS_TRANSMIT_ERROR;
  }
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

#ifndef USE_I2CWIRE
ISR(TWI_vect) {
  I2CManager.handleInterrupt();
}

#endif
