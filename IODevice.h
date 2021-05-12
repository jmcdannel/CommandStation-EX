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

// Define symbol DIAG_IO to enable diagnostic output
//#define DIAG_IO Y

// Define symbol IO_NO_HAL to reduce FLASH footpring when HAL features not required
#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO) 
#define IO_NO_HAL
#endif

#include "DIAG.h"
#include "FSH.h"
#include "I2CManager.h"

typedef uint16_t VPIN;
// Limit VPIN number to max 32767.  Above this number, printing often gives negative values.
// This should be enough for 99% of users.
#define VPIN_MAX 32767  
#define VPIN_NONE 65535


typedef void IONotifyStateChangeCallback(VPIN vpin, int value);


/*
 * IODevice class
 * 
 * This class is the basis of the Hardware Abstraction Layer (HAL) for
 * the DCC++EX Command Station.  All device classes derive from this.
 * 
 */

class IODevice {
public:

  // Parameter values to identify type of call to IODevice::configure.
  typedef enum : uint8_t {
    CONFIGURE_INPUT = 1,
    CONFIGURE_SERVO = 2,
    CONFIGURE_OUTPUT = 3,
  } ConfigTypeEnum;

  typedef enum : uint8_t {
    DEVSTATE_DORMANT = 0,
    DEVSTATE_PROBING = 1,
    DEVSTATE_INITIALISING = 2,
    DEVSTATE_NORMAL = 3,
    DEVSTATE_SCANNING = 4,
  } DeviceStateEnum;

  // Static functions to find the device and invoke its member functions

  // begin is invoked to create any standard IODevice subclass instances
  static void begin();

  // configure is used invoke an IODevice instance's _configure method
  static bool configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);

  // write invokes the IODevice instance's _write method.
  static void write(VPIN vpin, int value);

  // check whether the pin supports notification.  If so, then regular _read calls are not required.
  static bool hasCallback(VPIN vpin);

  // read invokes the IODevice instance's _read method.
  static bool read(VPIN vpin);

  // loop invokes the IODevice instance's _loop method.
  static void loop();

  static void DumpAll();

  // exists checks whether there is a device owning the specified vpin
  static bool exists(VPIN vpin);

  // remove deletes the device associated with the vpin, if it is deletable
  static void remove(VPIN vpin);

  // Enable shared interrupt on specified pin for GPIO extender modules.  The extender module
  // should pull down this pin when requesting a scan.  The pin may be shared by multiple modules.
  void setGPIOInterruptPin(int16_t pinNumber);

  // Method to add a notification.  it is the caller's responsibility to save the return value
  // and invoke the event handler associate with it.  Example:
  //
  //    NotifyStateChangeCallback *nextEv = registerInputChangeNotification(myProc);
  //
  //    void processChange(VPIN pin, int value) {
  //      // Do something
  //      // Pass on to next event handler
  //      if (nextEv) nextEv(pin, value);
  //     }
  //
  static IONotifyStateChangeCallback *registerInputChangeNotification(IONotifyStateChangeCallback *callback);
  
protected:
  
  // Method to perform initialisation of the device (optionally implemented within device class)
  virtual void _begin() {}

  // Method to configure device (optionally implemented within device class)
  virtual bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) { 
    (void)vpin; (void)configType; (void)paramCount; (void)params; // Suppress compiler warning.
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

  // Function called to check whether callback notification is supported by this pin.
  //  Defaults to no, if not overridden by the device.
  //  The same value should be returned by all pins on the device, so only one need
  //  be checked.
  virtual bool _hasCallback(VPIN vpin) { 
    (void) vpin;
    return false; 
  }

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

  // Pin number of interrupt pin for GPIO extender devices.  The device will pull this
  //  pin low if an input changes state.
  int16_t _gpioInterruptPin = -1;

  // Static support function for subclass creation
  static void addDevice(IODevice *newDevice);

  // Notification of change
  static IONotifyStateChangeCallback *_notifyCallbackChain;

  DeviceStateEnum _deviceState = DEVSTATE_DORMANT;

private:
  // Method to check whether the vpin corresponds to this device
  bool owns(VPIN vpin);
  // Method to find device handling Vpin
  static IODevice *findDevice(VPIN vpin);

  IODevice *_nextDevice = 0;
  static IODevice *_firstDevice;

  static IODevice *_nextLoopDevice;

  
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for PCA9685 16-channel PWM module.
 */
 
class PCA9685 : public IODevice {
public:
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress);

private:
  // Constructor
  PCA9685(VPIN vpin, int nPins, uint8_t I2CAddress);
  // Device-specific initialisation
  void _begin();
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  void _display();

  uint8_t _I2CAddress; // 0x40-0x43 possible

  // structures for setting up non-blocking writes to servo controller
  I2CRB requestBlock;
  uint8_t outputBuffer[5];
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Example of an IODevice subclass for PCF8574 8-bit I/O expander.
 */
 
class PCF8574 : public IODevice {
public:
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress) ;

private:
  // Constructor
  PCF8574(VPIN vpin, int nPins, uint8_t I2CAddress);  
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration function.  
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  // Device-specific read function.
  int _read(VPIN vpin);
  void _display();
  void _loop(unsigned long currentMicros);
  // Address may be 0x20 up to 0x27, but this may conflict with an LCD if present on 0x27
  //  or an Adafruit LCD backpack which defaults to 0x20
  uint8_t _I2CAddress; 
  uint8_t _portInputState; 
  uint8_t _portOutputState;
  // Interval between refreshes of each input port
  static const int _portTickTime = 4000;
  unsigned long _lastLoopEntry = 0;

  I2CRB requestBlock;
  uint8_t inputBuffer[1]={0xff};  // One eight-bit register.
  uint8_t outputBuffer[1]; // Register number
  bool scanActive = false;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for MCP23017 16-bit I/O expander.
 */
 
class MCP23017 : public IODevice {
public:
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin=-1);
  
private:
  // Constructor
  MCP23017(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin=-1);
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
  // Device-specific write function.
  void _write(VPIN vpin, int value);

  // Function called to check whether callback notification is supported by this pin.
  bool _hasCallback(VPIN vpin);

  // Device-specific read function.
  int _read(VPIN vpin);
  // Device-specific loop function
  void _loop(unsigned long currentMicros);
  // Device specific identification function
  void _display();

  uint8_t _I2CAddress;
  uint16_t _currentPortState;  // GPIOA in LSB and GPIOB in MSB
  uint16_t _portMode;
  uint16_t _portPullup;  
  uint8_t _portCounter;
  // Interval between refreshes of each input port
  static const int _portTickTime = 4000;
  unsigned long _lastLoopEntry = 0;
  bool scanActive = false;

  I2CRB requestBlock;
  uint8_t inputBuffer[2]={0xff,0xff};  // Two eight-bit registers.
  uint8_t outputBuffer[1]; // Register number

  enum {
    REG_IODIRA = 0x00,
    REG_IODIRB = 0x01,
    REG_GPINTENA = 0x04,
    REG_GPINTENB = 0x05,
    REG_IOCON = 0x0A,
    REG_GPPUA = 0x0C,
    REG_GPPUB = 0x0D,
    REG_GPIOA = 0x12,
    REG_GPIOB = 0x13,
  };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for MCP23008 8-bit I/O expander.
 */
 
class MCP23008 : public IODevice {
public:
  static void create(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin=-1) ;

private:
  // Constructor
  MCP23008(VPIN vpin, int nPins, uint8_t I2CAddress, int interruptPin=-1);  
  // Device-specific initialisation
  void _begin();
  // Device-specific pin configuration
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  // Device-specific read function.
  int _read(VPIN vpin);
  void _display();
  void _loop(unsigned long currentMicros);

  // Address may be 0x20 up to 0x27, but this may conflict with an LCD if present on 0x27
  //  or an Adafruit LCD backpack which defaults to 0x20
  uint8_t _I2CAddress; 
  // Module state
  uint8_t _portDirection;
  uint8_t _portPullup;
  uint8_t _portInputState; 
  uint8_t _portOutputState;
  uint8_t _portCounter;
  bool scanActive = false;
  // Interval between successive input port scan cycles
  static const int _portTickTime = 4000;
  unsigned long _lastLoopEntry = 0;

  I2CRB requestBlock;
  int8_t currentPollDevice = -1;
  uint8_t outputBuffer[1]; // Register number
  uint8_t inputBuffer[1] = {0xff};  // One 8-bit register value

  enum {
    // Register definitions for MCP23008
    REG_IODIR=0x00,
    REG_GPPU=0x06,
    REG_GPIO=0x09
  };

};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for DCC accessory decoder.
 */
 
class DCCAccessoryDecoder: public IODevice {
public:
  static void create(VPIN firstVpin, int nPins, int DCCAddress, int DCCSubaddress);
  static void create(VPIN firstVpin, int nPins, int DCCLinearAddress);

private:
  // Constructor
  DCCAccessoryDecoder(VPIN firstVpin, int nPins, int DCCLinearAddress);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  void _display();
  int _dccLinearAddress;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
 *  IODevice subclass for arduino input/output pins.
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
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
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