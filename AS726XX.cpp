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

  Wrapper class to interchangeably use SparkFun AS7262/AS7263/AS7265X
 */

#include "AS726XX.h"

#ifndef _AS726X_h
// necessary to compile.
#define AS726X_ADDR 0x49 // 7-bit unshifted default I2C Address
#define SENSORTYPE_AS7262 0x3E
#define SENSORTYPE_AS7263 0x3F
#define AS726x_DEVICE_TYPE 0x00
#define AS726x_HW_VERSION 0x01
#endif

#ifndef _ADAFRUIT_AS7341_H
// necessary to compile.
#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI
#define AS7341_GAIN_4X AS7341_GAIN_1X
#define AS7341_GAIN_16X AS7341_GAIN_1X
#define AS7341_GAIN_64X AS7341_GAIN_1X
#endif

AS726XX::AS726XX() {
  dev5 = nullptr;
  dev2 = dev3 = nullptr;
  dev4 = nullptr;
  _i2cPort = nullptr;
  maxCh = 0;
  use_calibrated = true;
  measurement_mode = AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT;
  measurement_start = false;
}

boolean AS726XX::begin(TwoWire &wirePort) {
  boolean ret = false;

  _i2cPort = &wirePort;
  _i2cPort->begin();

  if (_isConnected(AS726X_ADDR)) {
    if (!dev5 && !dev2 && !dev3) {
      // This gives the H/W version for AS7265X and the type of sensor for
      // AS7262/3
      uint8_t val = virtualReadRegister(AS7265X_HW_VERSION_LOW);

      switch (val) {
      case SENSORTYPE_AS7262:
#ifdef AS726X_INCLUDED
        dev2 = new AS726X;
        maxCh = 6;
#endif
        break;
      case SENSORTYPE_AS7263:
#ifdef AS726X_INCLUDED
        dev3 = new AS726X;
        maxCh = 6;
#endif
        break;
      default:
        dev5 = new AS7265X;
        maxCh = 18;
      }
    }

    if (dev5)
      ret = dev5->begin(wirePort);
    if (dev2)
      ret = dev2->begin(wirePort);
    if (dev3)
      ret = dev3->begin(wirePort);
  } else if (_isConnected(AS7341_I2CADDR_DEFAULT)) {
    if (!dev4) {
#ifdef AS7341_INCLUDED
      dev4 = new Adafruit_AS7341;
      maxCh = 9;
#endif
    }

    if (dev4)
      ret = dev4->begin(AS7341_I2CADDR_DEFAULT, &wirePort);

    if (ret) {
      // AS7341 measures in 2.8 microsec while AS726x/AS7265x run in 2.8 millisec.
      // ATime x 1000 will give approx same sampling time to AS726x.
      dev4->setATIME(999);
      dev4->setASTEP(10);
    }
  }

  // Populate nm, getu, getc
  if (ret) {
    uint16_t v = 0;
    int i = 0;
    if (v = getAnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getBnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getCnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getDnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getEnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getFnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getGnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getHnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getInm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getJnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getKnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getLnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getRnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getSnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getTnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getUnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getVnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
    if (v = getWnm()) {
      nm[i] = v;
      readings[i] = 0;
      cal_params[i++] = 1.0;
    }
  }
  return ret;
}

boolean AS726XX::isConnected() {
  if (dev5)
    return dev5->isConnected();
  if (dev4)
    return _isConnected(AS7341_I2CADDR_DEFAULT);
  return _isConnected(AS726X_ADDR);
}

boolean AS726XX::_isConnected(uint8_t addr) {
  // AS7262/7263 Library does not have it. Use code in AS7265X library
  if (!_i2cPort)
    return false; // You call it before begin().

  _i2cPort->beginTransmission(addr);
  if (_i2cPort->endTransmission() != 0)
    return false; // Sensor did not ACK
  return true;
}

uint8_t AS726XX::getDeviceType() {
  if (dev5)
    return dev5->getDeviceType();
  if (dev2 || dev3)
    return virtualReadRegister(AS726x_DEVICE_TYPE);
  if (dev4)
    return AS7341_CHIP_ID;
  return 0;
}

uint8_t AS726XX::getHardwareVersion() {
  if (dev5)
    return dev5->getHardwareVersion();
  if (dev2 || dev3)
    return virtualReadRegister(AS726x_HW_VERSION);
  return 0;
}

#define AS726x_FW_VERSION_HIGH 0x02
#define AS726x_FW_VERSION_LOW 0x03

uint8_t AS726XX::getMajorFirmwareVersion() {
  if (dev5)
    return dev5->getMajorFirmwareVersion();
  if (dev2 || dev3)
    return virtualReadRegister(AS726x_FW_VERSION_LOW) >> 4;
  return 0;
}

uint8_t AS726XX::getPatchFirmwareVersion() {
  if (dev5)
    return dev5->getPatchFirmwareVersion();
  if (dev2 || dev3)
    return ((virtualReadRegister(AS726x_FW_VERSION_LOW) & 0x0F) << 2) |
           (virtualReadRegister(AS726x_FW_VERSION_HIGH) >> 6);
  return 0;
}

uint8_t AS726XX::getBuildFirmwareVersion() {
  if (dev5)
    return dev5->getBuildFirmwareVersion();
  if (dev2 || dev3)
    return virtualReadRegister(AS726x_FW_VERSION_LOW & 0x3F);
  return 0;
}

uint8_t AS726XX::getTemperature(uint8_t deviceNumber) {
  ; // Get temp in C of the master IC
  if (dev5)
    return dev5->getTemperature(deviceNumber);
  if (dev2)
    return dev2->getTemperature();
  if (dev3)
    return dev3->getTemperature();
  return 0;
}

float AS726XX::getTemperatureAverage() {
  if (dev5)
    return dev5->getTemperatureAverage();
  if (dev2)
    return dev2->getTemperature();
  if (dev3)
    return dev3->getTemperature();
  return 0.0;
}

void AS726XX::takeMeasurements() {
  if (dev4) {
    if (!measurement_start)
      dev4->enableSpectralMeasurement(true);
    measurement_start = true;
    setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT); //Set mode to all 6-channels
    while (!dataAvailable()) delay(1);
  } else {
      if (dev5)
          dev5->takeMeasurements();
      if (dev2)
          dev2->takeMeasurements();
      if (dev3)
          dev3->takeMeasurements();
      populateReadings();
  }
}

void AS726XX::takeMeasurementsWithBulb() {
  if (dev4) {
      enableBulb(AS7265x_LED_WHITE);
      takeMeasurements();
      disableBulb(AS7265x_LED_WHITE);
  } else {
      if (dev5)
          dev5->takeMeasurementsWithBulb();
      if (dev2)
          dev2->takeMeasurementsWithBulb();
      if (dev3)
          dev3->takeMeasurementsWithBulb();
      populateReadings();
  }
}

void AS726XX::enableIndicator() {
  if (dev5)
    dev5->enableIndicator();
  if (dev2)
    dev2->enableIndicator();
  if (dev3)
    dev3->enableIndicator();
}

void AS726XX::disableIndicator() {
  if (dev5)
    dev5->disableIndicator();
  if (dev2)
    dev2->disableIndicator();
  if (dev3)
    dev3->disableIndicator();
}

void AS726XX::enableBulb(uint8_t device) {
  if (dev5)
    dev5->enableBulb(device);
  else if (device == AS7265x_LED_WHITE && dev2)
    dev2->enableBulb();
  else if (device == AS7265x_LED_IR && dev3)
    dev3->enableBulb();
  else if (device == AS7265x_LED_WHITE && dev4)
    dev4->enableLED(true);
}

void AS726XX::disableBulb(uint8_t device) {
  if (dev5)
    dev5->disableBulb(device);
  else if (device == AS7265x_LED_WHITE && dev2)
    dev2->disableBulb();
  else if (device == AS7265x_LED_IR && dev3)
    dev3->disableBulb();
  else if (device == AS7265x_LED_WHITE && dev4)
    dev4->enableLED(false);
}

boolean AS726XX::isBulbAvailable(uint8_t device) {
  switch (device) {
  case AS7265x_LED_WHITE:
    return isWhiteBulbAvailable();
    break;
  case AS7265x_LED_IR:
    return isIRBulbAvailable();
    break;
  case AS7265x_LED_UV:
    return isUVBulbAvailable();
    break;
  }
  return false;
}

void AS726XX::setGain(uint8_t gain) {
  if (dev4) {
    dev4->setGain(static_cast<as7341_gain_t>(gain));
  } else {
      if (gain > AS7341_GAIN_64X)
          gain = AS7341_GAIN_64X;
      if (dev5)
          dev5->setGain(gain);
      if (dev2)
          dev2->setGain(gain);
      if (dev3)
          dev3->setGain(gain);
  }
}

void AS726XX::setMeasurementMode(uint8_t mode) {
  if (dev5)
    dev5->setMeasurementMode(mode);
  if (dev2)
    dev2->setMeasurementMode(mode);
  if (dev3)
    dev3->setMeasurementMode(mode);
  if (dev4) {
    measurement_mode = mode;
    dev4->startReading();
  }
}

void AS726XX::setIntegrationCycles(uint8_t cycleValue) {
  if (dev5)
    dev5->setIntegrationCycles(cycleValue);
  if (dev2)
    dev2->setIntegrationTime(cycleValue);
  if (dev3)
    dev3->setIntegrationTime(cycleValue);
  if (dev4)
    dev4->setASTEP(cycleValue);
}

void AS726XX::setBulbCurrent(uint8_t current, uint8_t device) {
  if (dev5)
    dev5->setBulbCurrent(current, device);
  else if (device == AS7265x_LED_WHITE && dev2)
    dev2->setBulbCurrent(current);
  else if (device == AS7265x_LED_IR && dev3)
    dev3->setBulbCurrent(current);
  else if (device == AS7265x_LED_WHITE && dev4) {
    uint16_t i = 0;
    switch (current) {
    case AS7265X_LED_CURRENT_LIMIT_12_5MA:
      i = 12;
      break;
    case AS7265X_LED_CURRENT_LIMIT_25MA:
      i = 25;
      break;
    case AS7265X_LED_CURRENT_LIMIT_50MA:
      i = 50;
      break;
    case AS7265X_LED_CURRENT_LIMIT_100MA:
      i = 100;
      break;
    }
    dev4->setLEDCurrent(i);
  }
}

void AS726XX::setIndicatorCurrent(uint8_t current) {
  if (dev5)
    dev5->setIndicatorCurrent(current);
  if (dev2)
    dev2->setIndicatorCurrent(current);
  if (dev3)
    dev3->setIndicatorCurrent(current);
}

void AS726XX::enableInterrupt() {
  if (dev5)
    dev5->enableInterrupt();
  if (dev2)
    dev2->enableInterrupt();
  if (dev3)
    dev3->enableInterrupt();
  if (dev4)
    dev4->enableSystemInterrupt(true);
}

void AS726XX::disableInterrupt() {
  if (dev5)
    dev5->disableInterrupt();
  if (dev2)
    dev2->disableInterrupt();
  if (dev3)
    dev3->disableInterrupt();
  if (dev4)
    dev4->enableSystemInterrupt(false);
}

void AS726XX::softReset() {
  if (dev5)
    dev5->softReset();
  if (dev2)
    dev2->softReset();
  if (dev3)
    dev3->softReset();
}

boolean AS726XX::dataAvailable() {
  boolean ret = false;
  if (dev5)
    ret = dev5->dataAvailable();
  if (dev2)
    ret = dev2->dataAvailable();
  if (dev3)
    ret = dev3->dataAvailable();
  if (dev4)
    ret = dev4->checkReadingProgress();
  if (ret) {
    populateReadings();
  }
  return ret;
}

void AS726XX::populateReadings() {
  if (dev5 || dev2 || dev3) {
    float *itr = readings;
    float *cal = cal_params;
    if (getAnm()) {
      if (use_calibrated)
        *itr++ = getCalibratedA() * *cal++;
      else
        *itr++ = getA();
    }
    if (getBnm()) {
      if (use_calibrated)
        *itr++ = getCalibratedB() * *cal++;
      else
          *itr++ = getB();
    }
    if (getCnm()) {
        if (use_calibrated)
            *itr++ = getCalibratedC() * *cal++;
        else
          *itr++ = getC();
      }
      if (getDnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedD() * *cal++;
        else
          *itr++ = getD();
      }
      if (getEnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedE() * *cal++;
        else
          *itr++ = getE();
      }
      if (getFnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedF() * *cal++;
        else
          *itr++ = getF();
      }
      if (getGnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedG() * *cal++;
        else
          *itr++ = getG();
      }
      if (getHnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedH() * *cal++;
        else
          *itr++ = getH();
      }
      if (getInm()) {
        if (use_calibrated)
          *itr++ = getCalibratedI() * *cal++;
        else
          *itr++ = getI();
      }
      if (getJnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedJ() * *cal++;
        else
          *itr++ = getJ();
      }
      if (getKnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedK() * *cal++;
        else
          *itr++ = getK();
      }
      if (getLnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedL() * *cal++;
        else
          *itr++ = getL();
      }
      if (getRnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedR() * *cal++;
        else
          *itr++ = getR();
      }
      if (getSnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedS() * *cal++;
        else
          *itr++ = getS();
      }
      if (getTnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedT() * *cal++;
        else
          *itr++ = getT();
      }
      if (getUnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedU() * *cal++;
        else
          *itr++ = getU();
      }
      if (getVnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedV() * *cal++;
        else
          *itr++ = getV();
      }
      if (getWnm()) {
        if (use_calibrated)
          *itr++ = getCalibratedW() * *cal++;
        else
          *itr++ = getW();
      }
  } else if (dev4) {
      uint16_t buf[12];
      dev4->getAllChannels(buf);
    int i = 0;
      float *itr = readings;
      float *cal = cal_params;
      for (int j=0; j<maxCh; itr++, cal++, i++, j++) {
        if (use_calibrated)
          *itr = *cal * buf[i];
        else
          *itr = buf[i];
        // i == 4 for clear, i == 5 for NIR.
        if (i == 3)
          i += 2;
        // i == 10 for clear, i == 11 for NIR.
        if (i == 9)
          i++;
      }
      if (measurement_mode == AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS)
          dev4->startReading();
  }
}

// Returns the various calibration data
float AS726XX::getCalibratedA() {
  if (dev5)
    return dev5->getCalibratedA();
  else
    return 0.0;
}

float AS726XX::getCalibratedB() {
  if (dev5)
    return dev5->getCalibratedB();
  else
    return 0.0;
}

float AS726XX::getCalibratedC() {
  if (dev5)
    return dev5->getCalibratedC();
  else
    return 0.0;
}

float AS726XX::getCalibratedD() {
  if (dev5)
    return dev5->getCalibratedD();
  else
    return 0.0;
}

float AS726XX::getCalibratedE() {
  if (dev5)
    return dev5->getCalibratedE();
  else
    return 0.0;
}

float AS726XX::getCalibratedF() {
  if (dev5)
    return dev5->getCalibratedF();
  else
    return 0.0;
}

float AS726XX::getCalibratedG() {
  if (dev5)
    return dev5->getCalibratedG();
  else
    return 0.0;
}

float AS726XX::getCalibratedH() {
  if (dev5)
    return dev5->getCalibratedH();
  else
    return 0.0;
}

float AS726XX::getCalibratedI() {
  if (dev5)
    return dev5->getCalibratedI();
  else
    return 0.0;
}

float AS726XX::getCalibratedJ() {
  if (dev5)
    return dev5->getCalibratedJ();
  else
    return 0.0;
}

float AS726XX::getCalibratedK() {
  if (dev5)
    return dev5->getCalibratedK();
  else
    return 0.0;
}

float AS726XX::getCalibratedL() {
  if (dev5)
    return dev5->getCalibratedL();
  else
    return 0.0;
}

float AS726XX::getCalibratedR() {
  if (dev5)
    return dev5->getCalibratedR();
  if (dev2)
    return dev2->getCalibratedR();
  if (dev3)
    return dev3->getCalibratedR();
  else
    return 0.0;
}

float AS726XX::getCalibratedS() {
  if (dev5)
    return dev5->getCalibratedS();
  if (dev2)
    return dev2->getCalibratedS();
  if (dev3)
    return dev3->getCalibratedS();
  else
    return 0.0;
}

float AS726XX::getCalibratedT() {
  if (dev5)
    return dev5->getCalibratedT();
  if (dev2)
    return dev2->getCalibratedT();
  if (dev3)
    return dev3->getCalibratedT();
  else
    return 0.0;
}

float AS726XX::getCalibratedU() {
  if (dev5)
    return dev5->getCalibratedU();
  if (dev2)
    return dev2->getCalibratedU();
  if (dev3)
    return dev3->getCalibratedU();
  else
    return 0.0;
}

float AS726XX::getCalibratedV() {
  if (dev5)
    return dev5->getCalibratedV();
  if (dev2)
    return dev2->getCalibratedV();
  if (dev3)
    return dev3->getCalibratedV();
  else
    return 0.0;
}

float AS726XX::getCalibratedW() {
  if (dev5)
    return dev5->getCalibratedW();
  if (dev2)
    return dev2->getCalibratedW();
  if (dev3)
    return dev3->getCalibratedW();
  else
    return 0.0;
}

// Get the various raw readings
uint16_t AS726XX::getA() {
  if (dev5)
    return dev5->getA();
  else
    return 0;
}

uint16_t AS726XX::getB() {
  if (dev5)
    return dev5->getB();
  else
    return 0;
}

uint16_t AS726XX::getC() {
  if (dev5)
    return dev5->getC();
  else
    return 0;
}

uint16_t AS726XX::getD() {
  if (dev5)
    return dev5->getD();
  else
    return 0;
}

uint16_t AS726XX::getE() {
  if (dev5)
    return dev5->getE();
  else
    return 0;
}

uint16_t AS726XX::getF() {
  if (dev5)
    return dev5->getF();
  else
    return 0;
}

uint16_t AS726XX::getG() {
  if (dev5)
    return dev5->getG();
  else
    return 0;
}

uint16_t AS726XX::getH() {
  if (dev5)
    return dev5->getH();
  else
    return 0;
}

uint16_t AS726XX::getI() {
  if (dev5)
    return dev5->getI();
  else
    return 0;
}

uint16_t AS726XX::getJ() {
  if (dev5)
    return dev5->getJ();
  else
    return 0;
}

uint16_t AS726XX::getK() {
  if (dev5)
    return dev5->getK();
  else
    return 0;
}

uint16_t AS726XX::getL() {
  if (dev5)
    return dev5->getL();
  else
    return 0;
}

uint16_t AS726XX::getR() {
  if (dev5)
    return dev5->getR();
  if (dev2)
    return dev2->getR();
  if (dev3)
    return dev3->getR();
  else
    return 0;
}

uint16_t AS726XX::getS() {
  if (dev5)
    return dev5->getS();
  if (dev2)
    return dev2->getS();
  if (dev3)
    return dev3->getS();
  else
    return 0;
}

uint16_t AS726XX::getT() {
  if (dev5)
    return dev5->getT();
  if (dev2)
    return dev2->getT();
  if (dev3)
    return dev3->getT();
  else
    return 0;
}

uint16_t AS726XX::getU() {
  if (dev5)
    return dev5->getU();
  if (dev2)
    return dev2->getU();
  if (dev3)
    return dev3->getU();
  else
    return 0;
}

uint16_t AS726XX::getV() {
  if (dev5)
    return dev5->getV();
  if (dev2)
    return dev2->getV();
  if (dev3)
    return dev3->getV();
  else
    return 0;
}

uint16_t AS726XX::getW() {
  if (dev5)
    return dev5->getW();
  if (dev2)
    return dev2->getW();
  if (dev3)
    return dev3->getW();
  else
    return 0;
}

// Used for device detection
// Copied from SparkFun_AS7265X.cpp

uint8_t AS726XX::virtualReadRegister(uint8_t virtualAddr) {
  uint8_t status;

  // Do a prelim check of the read register
  status = readRegister(AS7265X_STATUS_REG);
  if ((status & AS7265X_RX_VALID) != 0) { // There is data to be read
    uint8_t incoming =
        readRegister(AS7265X_READ_REG); // Read the byte but do nothing with it
  }

  // Wait for WRITE flag to clear
  while (1) {
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_TX_VALID) == 0)
      break; // If TX bit is clear, it is ok to write
    delay(AS7265X_POLLING_DELAY);
  }

  // Send the virtual register address (bit 7 should be 0 to indicate we are
  // reading a register).
  writeRegister(AS7265X_WRITE_REG, virtualAddr);

  // Wait for READ flag to be set
  while (1) {
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_RX_VALID) != 0)
      break; // Read data is ready.
    delay(AS7265X_POLLING_DELAY);
  }

  uint8_t incoming = readRegister(AS7265X_READ_REG);
  return (incoming);
}

// Reads from a give location from the AS726x
uint8_t AS726XX::readRegister(uint8_t addr) {
  _i2cPort->beginTransmission(AS7265X_ADDR);
  _i2cPort->write(addr);
  if (_i2cPort->endTransmission() != 0) {
    // Serial.println("No ack!");
    return (0); // Device failed to ack
  }

  _i2cPort->requestFrom((uint8_t)AS7265X_ADDR, (uint8_t)1);
  if (_i2cPort->available()) {
    return (_i2cPort->read());
  }

  // Serial.println("No ack!");
  return (0); // Device failed to respond
}

// Write a value to a spot in the AS726x
boolean AS726XX::writeRegister(uint8_t addr, uint8_t val) {
  _i2cPort->beginTransmission(AS7265X_ADDR);
  _i2cPort->write(addr);
  _i2cPort->write(val);
  if (_i2cPort->endTransmission() != 0) {
    // Serial.println("No ack!");
    return (false); // Device failed to ack
  }

  return (true);
}
