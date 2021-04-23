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

#define  SENSOR_DECAY  0.03

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
  byte latchdelay;
  Sensor *nextSensor;
  static void load();
  static void store();
  static Sensor *create(int, int, int);
  static Sensor* get(int);  
  static bool remove(int);  
  static void checkAll(Print *);
  static void printAll(Print *);
  static unsigned int lastReadCycle; // low 16 bits of micros, holds up to 64 milliseconds
  static const unsigned int cycleInterval = 2000; // min time between consecutive reads of each sensor in microsecs.
  static const unsigned int minReadCount = 2; // number of consecutive reads before acting on change
                                        // E.g. 2 x 2000 means debounce time of 4ms
}; // Sensor

#endif
