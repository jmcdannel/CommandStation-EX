/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * @file RF24.h
 *
 * Class declaration for RF24 and helper enums
 */

#ifndef __RF24_H__
#define __RF24_H__

#define FAILURE_HANDLING
#define MINIMAL
#define RF24_POWERUP_DELAY	5000
#define rf24_max(a, b) (a>b?a:b)
#define rf24_min(a, b) (a<b?a:b)

#define RF24_SPI_SPEED 10000000

#include <SPI.h>
#define _SPI SPIClass

typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
     RF24_PA_HIGH,
     RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;

typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

typedef enum {
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;


class RF24 {
private:
    _SPI* _spi;
    uint16_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
    uint16_t csn_pin; /**< SPI Chip select */
    uint32_t spi_speed; /**< SPI Bus Speed */
    uint8_t status; /** The status byte returned from every SPI transaction */
    uint8_t payload_size; /**< Fixed size of payloads */
    bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
    bool ack_payloads_enabled; /**< Whether ack payloads are enabled. */
    uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
    uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */
    uint8_t config_reg; /**< For storing the value of the NRF_CONFIG register */
    bool _is_p_variant; /** For storing the result of testing the toggleFeatures() affect */


protected:
    inline void beginTransaction();
    inline void endTransaction();

public:

    RF24(uint16_t _cepin, uint16_t _cspin, uint32_t _spi_speed = RF24_SPI_SPEED);
    RF24(uint32_t _spi_speed = RF24_SPI_SPEED);

    bool begin(void);
    bool begin(uint16_t _cepin, uint16_t _cspin);
    bool begin(_SPI* spiBus);
    bool begin(_SPI* spiBus, uint16_t _cepin, uint16_t _cspin);
    bool isChipConnected();
    void startListening(void);
    void stopListening(void);
    bool available(void);
    void read(void* buf, uint8_t len);
    bool write(const void* buf, uint8_t len);
    void openWritingPipe(const uint8_t* address);
    void openReadingPipe(uint8_t number, const uint8_t* address);
    void printDetails(void);
    void printPrettyDetails(void);
    bool available(uint8_t* pipe_num);
    bool rxFifoFull();
    void powerDown(void);
    void powerUp(void);
    bool write(const void* buf, uint8_t len, const bool multicast);
    bool writeFast(const void* buf, uint8_t len);
    bool writeFast(const void* buf, uint8_t len, const bool multicast);
    bool writeBlocking(const void* buf, uint8_t len, uint32_t timeout);
    bool isWriteFinished( bool &ok);
    bool txStandBy();
    bool txStandBy(uint32_t timeout, bool startTx = 0);
    bool writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
    void whatHappened(bool& tx_ok, bool& tx_fail, bool& rx_ready);
    void startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx = 1);
    bool startWrite(const void* buf, uint8_t len, const bool multicast);
    void reUseTX();
    uint8_t flush_tx(void);
    uint8_t flush_rx(void);
    bool testCarrier(void);
    bool testRPD(void);
    bool isValid();
    void closeReadingPipe(uint8_t pipe);
    bool failureDetected;
    void setAddressWidth(uint8_t a_width);
    void setRetries(uint8_t delay, uint8_t count);
    void setChannel(uint8_t channel);
    uint8_t getChannel(void);
    void setPayloadSize(uint8_t size);
    uint8_t getPayloadSize(void);
    uint8_t getDynamicPayloadSize(void);
    void enableAckPayload(void);
    void disableAckPayload(void);
    void enableDynamicPayloads(void);
    void disableDynamicPayloads(void);
    void enableDynamicAck();
    bool isPVariant(void);
    void setAutoAck(bool enable);
    void setAutoAck(uint8_t pipe, bool enable);
    void setPALevel(uint8_t level, bool lnaEnable = 1);
    uint8_t getPALevel(void);
    uint8_t getARC(void);
    bool setDataRate(rf24_datarate_e speed);
    rf24_datarate_e getDataRate(void);
    void setCRCLength(rf24_crclength_e length);
    rf24_crclength_e getCRCLength(void);
    void disableCRC(void);
    void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);
    uint32_t txDelay;
    uint32_t csDelay;
    void startConstCarrier(rf24_pa_dbm_e level, uint8_t channel);
    void stopConstCarrier(void);
    void openReadingPipe(uint8_t number, uint64_t address);
    void openWritingPipe(uint64_t address);
    bool isAckPayloadAvailable(void);

private:
    void _init_obj();
    bool _init_radio();
    bool _init_pins();
    void csn(bool mode);
    void ce(bool level);
    void read_register(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, const uint8_t* buf, uint8_t len);
    void write_register(uint8_t reg, uint8_t value, bool is_cmd_only = false);
    void write_payload(const void* buf, uint8_t len, const uint8_t writeType);
    void read_payload(void* buf, uint8_t len);
    uint8_t get_status(void);

    void print_status(uint8_t status);
    void print_observe_tx(uint8_t value);
    void print_byte_register(const char* name, uint8_t reg, uint8_t qty = 1);
    void print_address_register(const char* name, uint8_t reg, uint8_t qty = 1);

    void toggle_features(void);
    void errNotify(void);

};

#endif // __RF24_H__
