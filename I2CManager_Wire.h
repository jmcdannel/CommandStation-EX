#include <Arduino.h>
#include <Wire.h>
#include "I2CManager.h"

void I2CManagerClass::_initialise() {
  Wire.begin();
}

void I2CManagerClass::_setClock(unsigned long i2cClockSpeed) {
  Wire.setClock(i2cClockSpeed);
}

/***************************************************************************
 *  Initiate a write to an I2C device (blocking operation on Wire)
 ***************************************************************************/
uint8_t I2CManagerClass::write(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb) {
  uint8_t status = I2C_STATUS_OK;
  Wire.beginTransmission(address);
  if (size > 0) Wire.write(buffer, size);
  status = Wire.endTransmission();
  return finishRB(rb, status);
}

/***************************************************************************
 *  Initiate a write from PROGMEM (flash) to an I2C device (blocking operation on Wire)
 ***************************************************************************/
uint8_t I2CManagerClass::write_P(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb) {
  uint8_t ramBuffer[size];
  const uint8_t *p1 = buffer;
  for (uint8_t i=0; i<size; i++)
    ramBuffer[i] = GETFLASH(p1++);
  uint8_t status = write(address, ramBuffer, size, rb);
  return finishRB(rb, status); 
}

/***************************************************************************
 *  Initiate a write (optional) followed by a read from the I2C device (blocking operation on Wire)
 *  If fewer than the number of requested bytes are received, status is I2C_STATUS_TRUNCATED.
 ***************************************************************************/
uint8_t I2CManagerClass::read(uint8_t address, uint8_t readBuffer[], uint8_t readSize,
                              const uint8_t writeBuffer[], uint8_t writeSize, I2CRB *rb)
{
  uint8_t status = I2C_STATUS_OK;
  uint8_t nBytes = 0;
  if (writeSize > 0) {
    Wire.beginTransmission(address);
    Wire.write(writeBuffer, writeSize);
    status = Wire.endTransmission(false); // Don't free bus yet
  }
  if (status == I2C_STATUS_OK) {
    Wire.requestFrom(address, (size_t)readSize);
    while (Wire.available() && nBytes < readSize) 
      readBuffer[nBytes++] = Wire.read();
    if (nBytes < readSize) rb->status = I2C_STATUS_TRUNCATED;
  }
  rb->nBytes = nBytes;
  return finishRB(rb, status);
}

/***************************************************************************
 *  Function to queue a request block and initiate operations.
 * 
 * For the Wire version, this executes synchronously, but the status is
 * returned in the I2CRB as for the asynchronous version.
 ***************************************************************************/
void I2CManagerClass::queueRequest(I2CRB *req) {
  uint8_t status;
  switch (req->operation) {
    case OPERATION_READ:
      status = read(req->i2cAddress, req->readBuffer, req->readLen);
      break;
    case OPERATION_SEND:
      status = write(req->i2cAddress, req->writeBuffer, req->writeLen);
      break;
    case OPERATION_SEND_P:
      status = write_P(req->i2cAddress, req->writeBuffer, req->writeLen);
      break;
    case OPERATION_REQUEST:
      status = read(req->i2cAddress, req->readBuffer, req->readLen, req->writeBuffer, req->writeLen);
      break;
  }
  req->status = status;
}

// Loop function
void I2CManagerClass::loop() {}

