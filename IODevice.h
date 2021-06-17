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

// Define symbol DIAG_LOOPTIMES to enable CS loop execution time to be reported
#define DIAG_LOOPTIMES

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
    DEVSTATE_FAILED = 5,
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
  // Without the shared interrupt, input states are scanned periodically to detect changes on 
  // GPIO extender pins.  If a shared interrupt pin is configured, then input states are scanned
  // only when the shared interrupt pin is pulled low.  The external GPIO module releases the pin
  // once the GPIO port concerned has been read.
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
  // Note that this implementation is rudimentary and assumes a small number of callbacks (typically one).  If 
  //  more than one callback is registered, then the calls to successive callback functions are
  //  nested, and stack usage will be impacted.  If callbacks are extensively used, it is recommended that
  //  a class or struct be implemented to hold the callback address, which can be chained to avoid
  //  nested callbacks.
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
  enum ProfileType {
    Instant = 0,  // Moves immediately between positions
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4    // For semaphores/turnouts with a bit of bounce!!
  };

private:
  // Constructor
  PCA9685(VPIN vpin, int nPins, uint8_t I2CAddress);
  // Device-specific initialisation
  void _begin();
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);
  // Device-specific write function.
  void _write(VPIN vpin, int value);
  void _loop(unsigned long currentMicros);
  void updatePosition(uint8_t pin);
  void writeDevice(uint8_t pin, int value);
  void _display();

  uint8_t _I2CAddress; // 0x40-0x43 possible

  struct ServoData {
    uint16_t activePosition : 12;
    uint16_t inactivePosition : 12;
    int8_t state;
    uint8_t profile;
    uint16_t currentPosition : 12;
    uint16_t fromPosition : 12;
    uint16_t toPosition : 12;
    uint8_t stepNumber;
    uint8_t numSteps;
  };
  struct ServoData *_servoData [16];

  static const uint8_t _catchupSteps = 5; // number of steps to wait before switching servo off
  static const byte FLASH _bounceProfile[30];

  const unsigned int refreshInterval = 50; // refresh every 50ms
  unsigned int _lastRefreshTime; // low 16-bits of millis() count.

  // structures for setting up non-blocking writes to servo controller
  I2CRB requestBlock;
  uint8_t outputBuffer[5];
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

// #include "IO_MCP23008.h"
// #include "IO_MCP23017.h"
// #include "IO_PCF8574.h"

#endif // iodevice_h