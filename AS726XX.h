/*
  The MIT License (MIT)
  Copyright (c) Kiyo Chinzei (kchinzei@gmail.com)
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/
/*
  AS726XX-CommonLib
  Kiyo Chinzei
  https://github.com/kchinzei/AS726XX-CommonLib

  Wrapper class to interchangeably use the following ICs with Sparkfun AS7265X.
  - SparkFun AS7262/AS7263
  - Adafruit AS7341
  AS7265X has 3 sensors/18 bands while AS7262/AS7263 one/6 bands, AS7341 one/10
  bands. By using `AS726XX` class, your code can treat AS7262/AS7263 or AS7341
  as if AS7265x is connected - single code works for all.

  Why not a child class of AS7265X?
  You can also write a class AS726XX as a child class of AS7265X.
  By doing so you can assure and inherit the interface of AS7265X.
  When you connect AS7262/AS7263, you manage an AS726X object separately.
  Simple. But that case allocates unused AS7265X object.
  I wanted memory footprint as small as possible.
 */

#ifndef _AS726XX_H
#define _AS726XX_H

#include <Arduino.h>
#include <Wire.h>

#include <AS726X.h> // if you don't need/have it, comment out this line.
#include <Adafruit_AS7341.h> // if you don't need/have it, comment out this line.
#include <SparkFun_AS7265X.h> // You need it.

#ifdef _AS726X_h
#define AS726X_INCLUDED
#endif

#ifdef _ADAFRUIT_AS7341_H
#define AS7341_INCLUDED
#endif

#define AS726XX_MAX_CH 18

class AS726X_dummy; // We define later
class AS7341_dummy; // We define later

class AS726XX {
public:
  AS726XX();

  /*
    Instead of using getA()/getCalibratedA() etc, you can do like
    AS726XX sensor;
    ...
    for (int i=0; i<sensor.maxCh; i++) {
      float val = sensor.readings[i];
      ...
  */
  uint8_t maxCh;
  uint16_t nm[AS726XX_MAX_CH];
  float readings[AS726XX_MAX_CH];
  float cal_params[AS726XX_MAX_CH]; // calibration parameter. 1.0 by default
  boolean use_calibrated;

  boolean begin(TwoWire &wirePort = Wire);
  boolean isConnected(); // Checks if sensor ack's the I2C request

  uint8_t getDeviceType();
  uint8_t getHardwareVersion();
  uint8_t getMajorFirmwareVersion();
  uint8_t getPatchFirmwareVersion();
  uint8_t getBuildFirmwareVersion();

  uint8_t getTemperature(uint8_t deviceNumber = 0);
  float getTemperatureAverage();

  void takeMeasurements();
  void takeMeasurementsWithBulb();

  void enableIndicator();
  void disableIndicator();

  void enableBulb(uint8_t device);
  void disableBulb(uint8_t device);

  void setGain(uint8_t gain); // 1 to 64x
  void setMeasurementMode(uint8_t mode);
  void setIntegrationCycles(uint8_t cycleValue);

  void setBulbCurrent(uint8_t current, uint8_t device);
  void setIndicatorCurrent(uint8_t current);

  void enableInterrupt();
  void disableInterrupt();

  void softReset();

  boolean dataAvailable(); // Returns true when data is available

  // Returns the various calibration data
  float getCalibratedA();
  float getCalibratedB();
  float getCalibratedC();
  float getCalibratedD();
  float getCalibratedE();
  float getCalibratedF();

  float getCalibratedG();
  float getCalibratedH();
  float getCalibratedI();
  float getCalibratedJ();
  float getCalibratedK();
  float getCalibratedL();

  float getCalibratedR();
  float getCalibratedS();
  float getCalibratedT();
  float getCalibratedU();
  float getCalibratedV();
  float getCalibratedW();

  // Get the various raw readings
  uint16_t getA();
  uint16_t getB();
  uint16_t getC();
  uint16_t getD();
  uint16_t getE();
  uint16_t getF();

  uint16_t getG();
  uint16_t getH();
  uint16_t getI();
  uint16_t getJ();
  uint16_t getK();
  uint16_t getL();

  uint16_t getR();
  uint16_t getS();
  uint16_t getT();
  uint16_t getU();
  uint16_t getV();
  uint16_t getW();

  // Original functions

  boolean isUVBulbAvailable() { return dev5 != nullptr; }
  boolean isWhiteBulbAvailable() {
    return dev5 != nullptr || dev2 != nullptr || dev4 != nullptr;
  }
  boolean isIRBulbAvailable() { return dev5 != nullptr || dev3 != nullptr; }
  boolean isBulbAvailable(uint8_t device);
  uint16_t getAnm() { return dev5 ? 410 : (dev4 ? 415 : 0); }
  uint16_t getBnm() { return dev5 ? 435 : (dev4 ? 445 : 0); }
  uint16_t getCnm() { return dev5 ? 460 : 0; }
  uint16_t getDnm() { return dev5 ? 485 : (dev4 ? 480 : 0); }
  uint16_t getEnm() { return dev5 ? 510 : (dev4 ? 515 : 0); }
  uint16_t getFnm() { return dev5 ? 535 : 0; }

  uint16_t getGnm() { return dev5 ? 560 : (dev4 ? 555 : 0); }
  uint16_t getHnm() { return dev5 ? 585 : (dev4 ? 590 : 0); }
  uint16_t getInm() { return dev5 ? 645 : (dev4 ? 630 : 0); }
  uint16_t getJnm() { return dev5 ? 705 : (dev4 ? 680 : 0); }
  uint16_t getKnm() { return dev5 ? 900 : (dev4 ? 910 : 0); }
  uint16_t getLnm() { return dev5 ? 940 : 0; }

  uint16_t getRnm() { return dev5 || dev3 ? 610 : (dev2 ? 450 : 0); }
  uint16_t getSnm() { return dev5 || dev3 ? 680 : (dev2 ? 500 : 0); }
  uint16_t getTnm() { return dev5 || dev3 ? 730 : (dev2 ? 550 : 0); }
  uint16_t getUnm() { return dev5 || dev3 ? 760 : (dev2 ? 570 : 0); }
  uint16_t getVnm() { return dev5 || dev3 ? 810 : (dev2 ? 600 : 0); }
  uint16_t getWnm() { return dev5 || dev3 ? 860 : (dev2 ? 650 : 0); }

  uint16_t getDeviceNumber();

private:
  AS7265X *dev5;
#ifdef AS726X_INCLUDED
  AS726X *dev2;
  AS726X *dev3;
#else
  AS726X_dummy *dev2;
  AS726X_dummy *dev3;
#endif
#ifdef AS7341_INCLUDED
  Adafruit_AS7341 *dev4;
#else
  AS7341_dummy *dev4;
#endif
  TwoWire *_i2cPort;

  uint8_t measurement_mode;
  boolean measurement_start;

  void populateReadings();
  boolean _isConnected(uint8_t addr);
  uint8_t virtualReadRegister(uint8_t virtualAddr);
  uint8_t readRegister(uint8_t i2c_addr, uint8_t addr);
  boolean writeRegister(uint8_t i2c_addr, uint8_t addr, uint8_t val);
};

#ifndef AS726X_INCLUDED
class AS726X_dummy : public AS726XX {
  // This class is to avoid compile error when AS726X.h not found.
  // This class is not implemeted, instantiated.
public:
  void enableBulb() {} // dummy; do nothing
  void disableBulb() {}
  void setBulbCurrent(uint8_t current) {}
  void setIntegrationTime(uint8_t a) {}
  float getCalibratedViolet() {}
  float getCalibratedBlue() {}
  float getCalibratedGreen() {}
  float getCalibratedYellow() {}
  float getCalibratedOrange() {}
  float getCalibratedRed() {}
  int getViolet() {}
  int getBlue() {}
  int getGreen() {}
  int getYellow() {}
  int getOrange() {}
  int getRed() {}
};
#endif

#ifndef AS7341_INCLUDED
typedef enum {
  AS7341_GAIN_1X,
} as7341_gain_t;

class AS7341_dummy : public AS726XX {
  // This class is to avoid compile error when Adafruit_AS7341.h not found.
  // This class is not implemeted, instantiated.
public:
  bool begin(uint8_t i2c_addr, TwoWire *wire, int32_t sensor_id = 0) {
  } // dummy; do nothing
  bool setASTEP(uint16_t astep_value) {}
  bool setATIME(uint8_t atime_value) {}
  bool setGain(as7341_gain_t gain_value) {}
  bool startReading(void) {}
  bool checkReadingProgress() {}
  bool getAllChannels(uint16_t *readings_buffer) {}
  void powerEnable(bool enable_power) {}
  bool enableSpectralMeasurement(bool enable_measurement) {}
  bool enableSystemInterrupt(bool enable_int) {}
  bool enableLED(bool enable_led) {}
  bool setLEDCurrent(uint16_t led_current_ma) {}
};
#endif

#endif
