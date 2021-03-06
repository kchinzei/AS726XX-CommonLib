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

  Wrapper class to interchangeably use SparkFun AS7262/AS7263 to AS7265X.
  AS7265X has 3 sensors while AS7262/AS7263 one.
  By using `AS726XX` class, your code can treat AS7262/AS7263 as if AS7265x is
  connected - single code works for all.

  Why not a child class of AS7265X?
  You can also write a class AS726XX as a child class of AS7265X.
  By doing so you can assure and inherit the interface of AS7265X.
  When you connect AS7262/AS7263, you manage an AS726X object separately.
  Simple. But that case allocates unused AS7265X object.
  I wanted memory footprint as small as possible.
 */

#ifndef _AS726XX_H
#define _AS726XX_H

#include "Arduino.h"
#include "Wire.h"

#include <SparkFun_AS7265X.h>
#include <AS726X.h> // if you don't have it, comment out this line.

#ifdef _AS726X_h
#define AS726X_INCLUDED
#else
// necessary to compile.
#define AS726X_ADDR 0x49 //7-bit unshifted default I2C Address
#define SENSORTYPE_AS7262 0x3E
#define SENSORTYPE_AS7263 0x3F
#define AS726x_DEVICE_TYPE 0x00
#define AS726x_HW_VERSION 0x01
#endif

class AS726X_dummy; // We define later

class AS726XX {
public:
    AS726XX();

    boolean begin(TwoWire &wirePort = Wire);
    boolean isConnected(); //Checks if sensor ack's the I2C request

    uint8_t getDeviceType();
    uint8_t getHardwareVersion();
    uint8_t getMajorFirmwareVersion();
    uint8_t getPatchFirmwareVersion();
    uint8_t getBuildFirmwareVersion();

    uint8_t getTemperature(uint8_t deviceNumber = 0); //Get temp in C of the master IC
    float getTemperatureAverage();

    void takeMeasurements();
    void takeMeasurementsWithBulb();

    void enableIndicator();
    void disableIndicator();

    void enableBulb(uint8_t device);
    void disableBulb(uint8_t device);

    void setGain(uint8_t gain); //1 to 64x
    void setMeasurementMode(uint8_t mode);
    void setIntegrationCycles(uint8_t cycleValue);

    void setBulbCurrent(uint8_t current, uint8_t device);
    void setIndicatorCurrent(uint8_t current);

    void enableInterrupt();
    void disableInterrupt();

    void softReset();

    boolean dataAvailable(); //Returns true when data is available

    //Returns the various calibration data
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

    //Get the various raw readings
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
    boolean isUVBulbAvailable()    { return dev5 != nullptr; }
    boolean isWhiteBulbAvailable() { return dev5 != nullptr || dev2 != nullptr; }
    boolean isIRBulbAvailable()    { return dev5 != nullptr || dev3 != nullptr; }
    boolean isBulbAvailable(uint8_t device);
    uint16_t getAnm() { return dev5? 410 : 0; }
    uint16_t getBnm() { return dev5? 435 : 0; }
    uint16_t getCnm() { return dev5? 460 : 0; }
    uint16_t getDnm() { return dev5? 485 : 0; }
    uint16_t getEnm() { return dev5? 510 : 0; }
    uint16_t getFnm() { return dev5? 535 : 0; }

    uint16_t getGnm() { return dev5? 560 : 0; }
    uint16_t getHnm() { return dev5? 585 : 0; }
    uint16_t getInm() { return dev5? 645 : 0; }
    uint16_t getJnm() { return dev5? 705 : 0; }
    uint16_t getKnm() { return dev5? 900 : 0; }
    uint16_t getLnm() { return dev5? 940 : 0; }

    uint16_t getRnm() { return dev2? 450 : 610; }
    uint16_t getSnm() { return dev2? 500 : 680; }
    uint16_t getTnm() { return dev2? 550 : 730; }
    uint16_t getUnm() { return dev2? 570 : 760; }
    uint16_t getVnm() { return dev2? 600 : 810; }
    uint16_t getWnm() { return dev2? 650 : 860; }

    /*
      Instead of using getA() etc, you can do like
      AS726XX sensor;
      ...
      for (auto itr=std::begin(sensor.getc); itr != std::end(sensor.getc); itr++)
        float val = (sensor.*(*itr))();
     */
    using uint16_t_fptr = uint16_t (AS726XX::*)();
    using float_fptr = float (AS726XX::*)();
    std::vector<uint16_t> nm;
    std::vector<uint16_t_fptr> getu;
    std::vector<float_fptr> getc;

private:
    AS7265X *dev5;
#ifdef AS726X_INCLUDED
    AS726X *dev2;
    AS726X *dev3;
#else
    AS726X_dummy *dev2;
    AS726X_dummy *dev3;
#endif
    TwoWire *_i2cPort;

    boolean _isConnected(uint8_t addr);
    uint8_t virtualReadRegister(uint8_t virtualAddr);
    uint8_t readRegister(uint8_t addr);
    boolean writeRegister(uint8_t addr, uint8_t val);
};

class AS726X_dummy: public AS726XX {
    // This class is to avoid compile error when AS726X.h not found.
    // This class is not implemeted, instantiated.
public:
    void enableBulb() {} // dummy; do nothing
    void disableBulb() {} // dummy; do nothing
    void setBulbCurrent(uint8_t current) {} // dummy; do nothing
    void setIntegrationTime(uint8_t a) {} // dummy; do nothing
};

#endif
