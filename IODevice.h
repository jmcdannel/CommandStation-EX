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

#ifndef iodevice_h
#define iodevice_h

// Define symbol to enable diagnostic output
//#define DIAG_IO Y

#include "DIAG.h"
#include "FSH.h"
#include "I2CManager.h"

typedef uint16_t VPIN;
#define VPIN_MAX 32767  // Above this number, printing gives negative values.  This should be high enough
#define VPIN_NONE 65535


enum DeviceType {
  Analogue = 0xDAC
};

class IODeviceType;
class IODevice;

/* 
 * IODeviceType class
 * 
 * This class supports the registration of device types and the creation
 * of devices.
 * 
 */

#ifdef IO_LATEBINDING
class IODeviceType {
public:
  IODeviceType(int deviceType) { _deviceType = deviceType; }
  int getDeviceType() { return _deviceType; }
  IODeviceType *_nextDeviceType = 0;
  IODevice *(*createFunction)(VPIN vpin);
private:
  int _deviceType = 0;
};
#endif

/*
 * IODevice class
 * 
 * This class is the basis of the Hardware Abstraction Layer (HAL) for
 * the DCC++EX Command Station.  All device classes derive from this.
 * 
 */

class IODevice {
public:

#ifdef IO_LATEBINDING
  // Class factory method for creating arbitrary device types
  static IODevice *create(int deviceTypeID, VPIN firstVpin, int paramCount, int params[]);
#endif

  // Static functions to find the device and invoke its member functions

  // begin is invoked to create any standard IODevice subclass instances
  static void begin();

  // configure is used invoke an IODevice instance's _configure method
  static bool configure(VPIN vpin, int paramCount, int params[]);

  // configurePullup is used invoke a GPIO IODevice instance's _configurePullup method
  static bool configurePullup(VPIN vpin, bool pullup);

  // write invokes the IODevice instance's _write method.
  static void write(VPIN vpin, int value);

  // read invokes the IODevice instance's _read method.
  static bool read(VPIN vpin);

  // loop invokes the IODevice instance's _loop method.
  static void loop();

  static void DumpAll();

  // exists checks whether there is a device owning the specified vpin
  static bool exists(VPIN vpin);

  // remove deletes the device associated with the vpin, if it is deletable
  static void remove(VPIN vpin);

  // When a turnout needs to allocate a vpin as its output, it allocates one using ID+turnoutVpinOffset.
  //  static const VPIN turnoutVpinOffset = 300; 

  // VPIN of first PCA9685 servo controller pin.  
  static const VPIN firstServoVpin = 100;
  
protected:
#ifdef IO_LATEBINDING
  // Method to register the device handler to the IODevice system (called from device's register() method)
  static void _registerDeviceType(int deviceTypeID, IODevice *createFunction(VPIN));
#endif

  // Method to perform initialisation of the device (optionally implemented within device class)
  virtual void _begin() {}

  // Method to configure device (optionally implemented within device class)
  virtual bool _configure(VPIN vpin, int paramCount, int params[]) { 
    (void)vpin; (void)paramCount; (void)params; // Suppress compiler warning.
    return false;
  };

  // Device-specific pin configuration for GPIOs (optionally implemented within device class)
  virtual bool _configurePullup(VPIN vpin, bool pullup) {
    (void)vpin; (void)pullup; // Suppress compiler warning
    return false;
  }; 

  // Method to write new state (optionally implemented within device class)
  virtual void _write(VPIN vpin, int value) {
    (void)vpin; (void)value;
  };

  // Method called from within a filter device to trigger its output (which may
  // have the same VPIN id as the input to the filter).  It works through the 
  // later devices in the chain only.
  void writeDownstream(VPIN vpin, int value);

  // Method to read pin state (optionally implemented within device class)
  virtual int _read(VPIN vpin) { 
    (void)vpin; 
    return 0;
  };

  // Method to perform updates on an ongoing basis (optionally implemented within device class)
  virtual void _loop(unsigned long currentMicros) {
    (void)currentMicros; // Suppress compiler warning.
  };

  // Method for displaying info on DIAG output (optionally implemented within device class)
  virtual void _display();

  // Destructor
  virtual ~IODevice() {};
  
  // isDeletable returns true if object is deletable (i.e. is not a base device driver).
  virtual bool _isDeletable();

  // Common object fields.
  VPIN _firstVpin;
  int _nPins;

  // Static support function for subclass creation
  static void addDevice(IODevice *newDevice);

private:
  // Method to check whether the vpin corresponds to this device
  bool owns(VPIN vpin);
  // Method to find device handling Vpin
  static IODevice *findDevice(VPIN vpin);

  IODevice *_nextDevice = 0;
  static IODevice *_firstDevice;

  // Chain of installed device driver types
  static IODeviceType *_firstDeviceType;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Example of an IODevice subclass for PCA9685 16-channel PWM module.
 */
 
class PCA9685 : public IODevice {
public:
  static IODevice *createInstance(VPIN vpin, int nPins, uint8_t I2CAddress);
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress);

private:
  // Constructor
  PCA9685();
  // Device-specific initialisation
  void _begin();
  bool _configure(VPIN vpin, int paramCount, int params[]);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  void _display();
  // Helper function
  void writeRegister(byte address, byte reg, byte value);

  uint8_t _I2CAddress; // 0x40-0x43 used
  // Number of modules configured
  uint8_t _nModules;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Example of an IODevice subclass for PCF8574 8-bit I/O expander.
 */
 
class PCF8574 : public IODevice {
public:
  static IODevice *createInstance(VPIN vpin, int nPins, uint8_t I2CAddress);
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress) ;

private:
  // Constructor
  PCF8574();  
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration function.  
  bool _configure(VPIN vpin, bool pullup);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  // Device-specific read function.
  int _read(VPIN vpin);
  void _display();
  void _loop(unsigned long currentMicros);
  // Address may be 0x20 up to 0x27, but this may conflict with an LCD if present on 0x27
  //  or an Adafruit LCD backpack which defaults to 0x20
  uint8_t _I2CAddress; 
  // Maximum number of PCF8574 modules supported.
  uint8_t _nModules;
  static const int _maxModules = 8;
  uint8_t *_portInputState; 
  uint8_t *_portOutputState;
  uint8_t *_portCounter;
  // Interval between ticks when counters are updated
  static const int _portTickTime = 500;
  // Number of ticks to elapse before cached port values expire.
  static const int _minTicksBetweenPortReads = 4;
  unsigned long _lastLoopEntry = 0;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Example of an IODevice subclass for MCP23017 16-bit I/O expander.
 */
 
class MCP23017 : public IODevice {
public:
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress);
  static IODevice *createInstance(VPIN vpin, int nPins, uint8_t I2CAddress);
  
private:
  // Constructor
  MCP23017();
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration
  bool _configurePullup(VPIN vpin, bool pullup);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  // Device-specific read function.
  int _read(VPIN vpin);
  // Device-specific loop function
  void _loop(unsigned long currentMicros);
  // Device specific identification function
  void _display();
  // Helper functions
  void writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t value) ;
  void writeRegister2(uint8_t I2CAddress, uint8_t reg, uint8_t valueA, uint8_t valueB) ;
  uint16_t readRegister2(uint8_t I2CAddress, uint8_t reg);

  uint8_t _I2CAddress;
  uint8_t _nModules;
  static const int _maxModules = 8;
  uint16_t *_currentPortState;  // GPIOA in LSB and GPIOB in MSB
  uint16_t *_portMode;
  uint16_t *_portPullup;
  uint8_t *_portCounter;
  // Interval between ticks when counters are updated
  static const int _portTickTime = 500;
  // Number of ticks to elapse before cached port values expire.
  static const int _minTicksBetweenPortReads = 4;
  unsigned long _lastLoopEntry = 0;

  enum {
    REG_IODIRA = 0x00,
    REG_IODIRB = 0x01,
    REG_IOCON = 0x0A,
    REG_GPPUA = 0x0C,
    REG_GPPUB = 0x0D,
    REG_GPIOA = 0x12,
    REG_GPIOB = 0x13
  };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Example of an IODevice subclass for MCP23008 8-bit I/O expander.
 */
 
class MCP23008 : public IODevice {
public:
  static IODevice *createInstance(VPIN vpin, int nPins, uint8_t I2CAddress);
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress) ;

private:
  // Constructor
  MCP23008();  
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration
  bool _configurePullup(VPIN vpin, bool pullup);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  // Device-specific read function.
  int _read(VPIN vpin);
  void _display();
  void _loop(unsigned long currentMicros);
  // Helper functions
  void writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t value) ;
  uint8_t readRegister(uint8_t I2CAddress, uint8_t reg);

  // Address may be 0x20 up to 0x27, but this may conflict with an LCD if present on 0x27
  //  or an Adafruit LCD backpack which defaults to 0x20
  uint8_t _I2CAddress; 
  // Maximum number of MCP23008 modules supported.
  uint8_t _nModules;
  static const int _maxModules = 8;
  // Arrays per module
  uint8_t *_portDirection;
  uint8_t *_portPullup;
  uint8_t *_portInputState; 
  uint8_t *_portOutputState;
  uint8_t *_portCounter;
  // Interval between ticks when counters are updated
  static const int _portTickTime = 500;
  // Number of ticks to elapse before cached port values expire.
  static const int _minTicksBetweenPortReads = 4;
  unsigned long _lastLoopEntry = 0;

  enum {
    // Register definitions for MCP23008
    REG_IODIR=0x00,
    REG_GPPU=0x06,
    REG_GPIO=0x09
  };

};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Example of an IODevice subclass for DCC accessory decoder.
 */
 
class DCCAccessoryDecoder: public IODevice {
public:
  static void create(VPIN firstVpin, int DCCAddress, int DCCSubaddress);
  static void create(VPIN firstVpin, int DCCLinearAddress);

private:
  // Constructor
  DCCAccessoryDecoder();
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  void _display();
  int _DCCAddress;
  int _DCCSubaddress;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
 *  Example of an IODevice subclass for arduino input/output pins.
 */
 
class ArduinoPins: public IODevice {
public:
  static void create(VPIN firstVpin, int nPins) {
    addDevice(new ArduinoPins(firstVpin, nPins));
  }
  
  // Constructor
  ArduinoPins(VPIN firstVpin, int nPins);

private:
  // Device-specific pin configuration
  bool _configurePullup(VPIN vpin, bool pullup);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  // Device-specific read function.
  int _read(VPIN vpin);
  void _display();

  uint8_t *_pinPullups;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "IO_AnalogueDevice.h"

#endif // iodevice_h