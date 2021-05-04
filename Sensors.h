/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include "IODevice.h"

struct SensorData {
  int snum;
  uint8_t pin;
  uint8_t pullUp;
};

struct Sensor{
  static Sensor *firstSensor;
  static Sensor *readingSensor;
  SensorData data;
  boolean active;
  boolean inputState;
  byte latchdelay;
  Sensor *nextSensor;
  void setState(int state);
  static void load();
  static void store();
  static Sensor *create(int id, VPIN vpin, int pullUp);
  static Sensor* get(int id);  
  static bool remove(int id);  
  static void checkAll(Print *stream);
  static void printAll(Print *stream);
  static unsigned long lastReadCycle; // value of micros at start of last read cycle
  static const unsigned int cycleInterval = 5000; // min time between consecutive reads of each sensor in microsecs.
                                                  // should not be less than device scan cycle time.
  static const unsigned int minReadCount = 2; // number of consecutive reads before acting on change
                                        // E.g. 2 x 5000 means debounce time of 10ms
}; // Sensor

#endif
