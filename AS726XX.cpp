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

AS726XX::AS726XX() {
    dev5 = nullptr;
    dev2 = dev3 = nullptr;
    _i2cPort = nullptr;
}


boolean AS726XX::begin(TwoWire &wirePort) {
    _i2cPort = &wirePort;
    _i2cPort->begin();
    if (!_isConnected(AS726X_ADDR)) {
      return false;
    }

    if (!dev5 && !dev2 && !dev3) {
        // This gives the H/W version for AS7265X and the type of sensor for AS7262/3
        uint8_t val = virtualReadRegister(AS7265X_HW_VERSION_LOW);

        switch (val) {
        case SENSORTYPE_AS7262:
#ifdef AS726X_INCLUDED
            dev2 = new AS726X;
#endif
            break;
        case SENSORTYPE_AS7263:
#ifdef AS726X_INCLUDED
            dev3 = new AS726X;
#endif
            break;
        default:
            dev5 = new AS7265X;
        }
    }

    boolean ret = false;
    if (dev5) ret = dev5->begin(wirePort);
    if (dev2) ret = dev2->begin(wirePort);
    if (dev3) ret = dev3->begin(wirePort);

    // Populate nm, getu, getc
    if (ret) {
        
        uint16_t v = 0;
        if (v = this->getAnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getA);
            getc.push_back(&AS726XX::getCalibratedA);
        }
        if (v = this->getBnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getB);
            getc.push_back(&AS726XX::getCalibratedB);
        }
        if (v = this->getCnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getC);
            getc.push_back(&AS726XX::getCalibratedC);
        }
        if (v = this->getDnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getD);
            getc.push_back(&AS726XX::getCalibratedD);
        }
        if (v = this->getEnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getE);
            getc.push_back(&AS726XX::getCalibratedE);
        }
        if (v = this->getFnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getF);
            getc.push_back(&AS726XX::getCalibratedF);
        }
        if (v = this->getGnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getG);
            getc.push_back(&AS726XX::getCalibratedG);
        }
        if (v = this->getHnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getH);
            getc.push_back(&AS726XX::getCalibratedH);
        }
        if (v = this->getInm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getI);
            getc.push_back(&AS726XX::getCalibratedI);
        }
        if (v = this->getJnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getJ);
            getc.push_back(&AS726XX::getCalibratedJ);
        }
        if (v = this->getKnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getK);
            getc.push_back(&AS726XX::getCalibratedK);
        }
        if (v = this->getLnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getL);
            getc.push_back(&AS726XX::getCalibratedL);
        }
        if (v = this->getRnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getR);
            getc.push_back(&AS726XX::getCalibratedR);
        }
        if (v = this->getSnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getS);
            getc.push_back(&AS726XX::getCalibratedS);
        }
        if (v = this->getTnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getT);
            getc.push_back(&AS726XX::getCalibratedT);
        }
        if (v = this->getUnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getU);
            getc.push_back(&AS726XX::getCalibratedU);
        }
        if (v = this->getVnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getV);
            getc.push_back(&AS726XX::getCalibratedV);
        }
        if (v = this->getWnm()) {
            nm.push_back(v);
            getu.push_back(&AS726XX::getW);
            getc.push_back(&AS726XX::getCalibratedW);
        }
    }
    return ret;
}

boolean AS726XX::isConnected() {
    if (dev5) return dev5->isConnected();
    return _isConnected(AS726X_ADDR);
}

boolean AS726XX::_isConnected(uint8_t addr) {
    // AS7262/7263 Library does not have it. Use code in AS7265X library
    if (!_i2cPort) return false; // You call it before begin().

    _i2cPort->beginTransmission(addr);
    if (_i2cPort->endTransmission() != 0)
        return false; //Sensor did not ACK
    return true;
}

uint8_t AS726XX::getDeviceType() {
    if (dev5) return dev5->getDeviceType();
    if (dev2 || dev3)
        return virtualReadRegister(AS726x_DEVICE_TYPE);
    return 0;
}

uint8_t AS726XX::getHardwareVersion() {
    if (dev5) return dev5->getHardwareVersion();
    if (dev2 || dev3)
        return virtualReadRegister(AS726x_HW_VERSION);
    return 0;
}

#define AS726x_FW_VERSION_HIGH 0x02
#define AS726x_FW_VERSION_LOW  0x03

uint8_t AS726XX::getMajorFirmwareVersion() {
    if (dev5) return dev5->getMajorFirmwareVersion();
    if (dev2 || dev3)
        return virtualReadRegister(AS726x_FW_VERSION_LOW) >> 4;
    return 0;
}

uint8_t AS726XX::getPatchFirmwareVersion() {
    if (dev5) return dev5->getPatchFirmwareVersion();
    if (dev2 || dev3)
        return ((virtualReadRegister(AS726x_FW_VERSION_LOW) & 0x0F) << 2) | (virtualReadRegister(AS726x_FW_VERSION_HIGH) >> 6);
    return 0;
}

uint8_t AS726XX::getBuildFirmwareVersion() {
    if (dev5) return dev5->getBuildFirmwareVersion();
    if (dev2 || dev3)
        return virtualReadRegister(AS726x_FW_VERSION_LOW & 0x3F);
    return 0;
}


uint8_t AS726XX::getTemperature(uint8_t deviceNumber) {
    ; //Get temp in C of the master IC
    if (dev5) return dev5->getTemperature(deviceNumber);
    if (dev2) return dev2->getTemperature();
    if (dev3) return dev3->getTemperature();
    return 0;
}

float AS726XX::getTemperatureAverage() {
    if (dev5) return dev5->getTemperatureAverage();
    if (dev2) return dev2->getTemperature();
    if (dev3) return dev3->getTemperature();
    return 0.0;
}

void AS726XX::takeMeasurements() {
    if (dev5) dev5->takeMeasurements();
    if (dev2) dev2->takeMeasurements();
    if (dev3) dev3->takeMeasurements();
}

void AS726XX::takeMeasurementsWithBulb() {
    if (dev5) dev5->takeMeasurementsWithBulb();
    if (dev2) dev2->takeMeasurementsWithBulb();
    if (dev3) dev3->takeMeasurementsWithBulb();
}

void AS726XX::enableIndicator() {
    if (dev5) dev5->enableIndicator();
    if (dev2) dev2->enableIndicator();
    if (dev3) dev3->enableIndicator();
}

void AS726XX::disableIndicator() {
    if (dev5) dev5->disableIndicator();
    if (dev2) dev2->disableIndicator();
    if (dev3) dev3->disableIndicator();
}

void AS726XX::enableBulb(uint8_t device) {
    if (dev5) dev5->enableBulb(device);
    else if (device == AS7265x_LED_WHITE && dev2)
        dev2->enableBulb();
    else if (device == AS7265x_LED_IR && dev3)
        dev3->enableBulb();
}

void AS726XX::disableBulb(uint8_t device) {
    if (dev5) dev5->disableBulb(device);
    else if (device == AS7265x_LED_WHITE && dev2)
        dev2->disableBulb();
    else if (device == AS7265x_LED_IR && dev3)
        dev3->disableBulb();
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
    if (dev5) dev5->setGain(gain);
    if (dev2) dev2->setGain(gain);
    if (dev3) dev3->setGain(gain);
}

void AS726XX::setMeasurementMode(uint8_t mode) {
    if (dev5) dev5->setMeasurementMode(mode);
    if (dev2) dev2->setMeasurementMode(mode);
    if (dev3) dev3->setMeasurementMode(mode);
}

void AS726XX::setIntegrationCycles(uint8_t cycleValue) {
    if (dev5) dev5->setIntegrationCycles(cycleValue);
    if (dev2) dev2->setIntegrationTime(cycleValue);
    if (dev3) dev3->setIntegrationTime(cycleValue);
}

void AS726XX::setBulbCurrent(uint8_t current, uint8_t device) {
    if (dev5) dev5->setBulbCurrent(current, device);
    else if (device ==  AS7265x_LED_WHITE && dev2)
        dev2->setBulbCurrent(current);
    else if (device == AS7265x_LED_IR && dev3)
        dev3->setBulbCurrent(current);
}

void AS726XX::setIndicatorCurrent(uint8_t current) {
    if (dev5) dev5->setIndicatorCurrent(current);
    if (dev2) dev2->setIndicatorCurrent(current);
    if (dev3) dev3->setIndicatorCurrent(current);
}

void AS726XX::enableInterrupt() {
    if (dev5) dev5->enableInterrupt();
    if (dev2) dev2->enableInterrupt();
    if (dev3) dev3->enableInterrupt();
}

void AS726XX::disableInterrupt() {
    if (dev5) dev5->disableInterrupt();
    if (dev2) dev2->disableInterrupt();
    if (dev3) dev3->disableInterrupt();
}

void AS726XX::softReset() {
    if (dev5) dev5->softReset();
    if (dev2) dev2->softReset();
    if (dev3) dev3->softReset();
}

boolean AS726XX::dataAvailable() {
    if (dev5) return dev5->dataAvailable();
    if (dev2) return dev2->dataAvailable();
    if (dev3) return dev3->dataAvailable();
    else return false;
}

//Returns the various calibration data
float AS726XX::getCalibratedA() {
    if (dev5) return dev5->getCalibratedA();
    else return 0.0;
}

float AS726XX::getCalibratedB() {
    if (dev5) return dev5->getCalibratedB();
    else return 0.0;
}

float AS726XX::getCalibratedC() {
    if (dev5) return dev5->getCalibratedC();
    else return 0.0;
}

float AS726XX::getCalibratedD() {
    if (dev5) return dev5->getCalibratedD();
    else return 0.0;
}

float AS726XX::getCalibratedE() {
    if (dev5) return dev5->getCalibratedE();
    else return 0.0;
}

float AS726XX::getCalibratedF() {
    if (dev5) return dev5->getCalibratedF();
    else return 0.0;
}


float AS726XX::getCalibratedG() {
    if (dev5) return dev5->getCalibratedG();
    else return 0.0;
}

float AS726XX::getCalibratedH() {
    if (dev5) return dev5->getCalibratedH();
    else return 0.0;
}

float AS726XX::getCalibratedI() {
    if (dev5) return dev5->getCalibratedI();
    else return 0.0;
}

float AS726XX::getCalibratedJ() {
    if (dev5) return dev5->getCalibratedJ();
    else return 0.0;
}

float AS726XX::getCalibratedK() {
    if (dev5) return dev5->getCalibratedK();
    else return 0.0;
}

float AS726XX::getCalibratedL() {
    if (dev5) return dev5->getCalibratedL();
    else return 0.0;
}


float AS726XX::getCalibratedR() {
    if (dev5) return dev5->getCalibratedR();
    if (dev2) return dev2->getCalibratedR();
    if (dev3) return dev3->getCalibratedR();
    else return 0.0;
}

float AS726XX::getCalibratedS() {
    if (dev5) return dev5->getCalibratedS();
    if (dev2) return dev2->getCalibratedS();
    if (dev3) return dev3->getCalibratedS();
    else return 0.0;
}

float AS726XX::getCalibratedT() {
    if (dev5) return dev5->getCalibratedT();
    if (dev2) return dev2->getCalibratedT();
    if (dev3) return dev3->getCalibratedT();
    else return 0.0;
}

float AS726XX::getCalibratedU() {
    if (dev5) return dev5->getCalibratedU();
    if (dev2) return dev2->getCalibratedU();
    if (dev3) return dev3->getCalibratedU();
    else return 0.0;
}

float AS726XX::getCalibratedV() {
    if (dev5) return dev5->getCalibratedV();
    if (dev2) return dev2->getCalibratedV();
    if (dev3) return dev3->getCalibratedV();
    else return 0.0;
}

float AS726XX::getCalibratedW() {
    if (dev5) return dev5->getCalibratedW();
    if (dev2) return dev2->getCalibratedW();
    if (dev3) return dev3->getCalibratedW();
    else return 0.0;
}


//Get the various raw readings
uint16_t AS726XX::getA() {
    if (dev5) return dev5->getA();
    else return 0;
}

uint16_t AS726XX::getB() {
    if (dev5) return dev5->getB();
    else return 0;
}

uint16_t AS726XX::getC() {
    if (dev5) return dev5->getC();
    else return 0;
}

uint16_t AS726XX::getD() {
    if (dev5) return dev5->getD();
    else return 0;
}

uint16_t AS726XX::getE() {
    if (dev5) return dev5->getE();
    else return 0;
}

uint16_t AS726XX::getF() {
    if (dev5) return dev5->getF();
    else return 0;
}


uint16_t AS726XX::getG() {
    if (dev5) return dev5->getG();
    else return 0;
}

uint16_t AS726XX::getH() {
    if (dev5) return dev5->getH();
    else return 0;
}

uint16_t AS726XX::getI() {
    if (dev5) return dev5->getI();
    else return 0;
}

uint16_t AS726XX::getJ() {
    if (dev5) return dev5->getJ();
    else return 0;
}

uint16_t AS726XX::getK() {
    if (dev5) return dev5->getK();
    else return 0;
}

uint16_t AS726XX::getL() {
    if (dev5) return dev5->getL();
    else return 0;
}


uint16_t AS726XX::getR() {
    if (dev5) return dev5->getR();
    if (dev2) return dev2->getR();
    if (dev3) return dev3->getR();
    else return 0;
}

uint16_t AS726XX::getS() {
    if (dev5) return dev5->getS();
    if (dev2) return dev2->getS();
    if (dev3) return dev3->getS();
    else return 0;
}

uint16_t AS726XX::getT() {
    if (dev5) return dev5->getT();
    if (dev2) return dev2->getT();
    if (dev3) return dev3->getT();
    else return 0;
}

uint16_t AS726XX::getU() {
    if (dev5) return dev5->getU();
    if (dev2) return dev2->getU();
    if (dev3) return dev3->getU();
    else return 0;
}

uint16_t AS726XX::getV() {
    if (dev5) return dev5->getV();
    if (dev2) return dev2->getV();
    if (dev3) return dev3->getV();
    else return 0;
}

uint16_t AS726XX::getW() {
    if (dev5) return dev5->getW();
    if (dev2) return dev2->getW();
    if (dev3) return dev3->getW();
    else return 0;
}



// Used for device detection
// Copied from SparkFun_AS7265X.cpp

uint8_t AS726XX::virtualReadRegister(uint8_t virtualAddr)
{
    uint8_t status;

    //Do a prelim check of the read register
    status = readRegister(AS7265X_STATUS_REG);
    if ((status & AS7265X_RX_VALID) != 0) { //There is data to be read
        uint8_t incoming = readRegister(AS7265X_READ_REG); //Read the byte but do nothing with it
    }

    //Wait for WRITE flag to clear
    while (1) {
        status = readRegister(AS7265X_STATUS_REG);
        if ((status & AS7265X_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
        delay(AS7265X_POLLING_DELAY);
    }

    // Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
    writeRegister(AS7265X_WRITE_REG, virtualAddr);

    //Wait for READ flag to be set
    while (1) {
        status = readRegister(AS7265X_STATUS_REG);
        if ((status & AS7265X_RX_VALID) != 0) break; // Read data is ready.
        delay(AS7265X_POLLING_DELAY);
    }

    uint8_t incoming = readRegister(AS7265X_READ_REG);
    return (incoming);
}

//Reads from a give location from the AS726x
uint8_t AS726XX::readRegister(uint8_t addr)
{
    _i2cPort->beginTransmission(AS7265X_ADDR);
    _i2cPort->write(addr);
    if (_i2cPort->endTransmission() != 0) {
        //Serial.println("No ack!");
        return (0); //Device failed to ack
    }

    _i2cPort->requestFrom((uint8_t)AS7265X_ADDR, (uint8_t)1);
    if (_i2cPort->available()) {
        return (_i2cPort->read());
    }

    //Serial.println("No ack!");
    return (0); //Device failed to respond
}

//Write a value to a spot in the AS726x
boolean AS726XX::writeRegister(uint8_t addr, uint8_t val)
{
    _i2cPort->beginTransmission(AS7265X_ADDR);
    _i2cPort->write(addr);
    _i2cPort->write(val);
    if (_i2cPort->endTransmission() != 0) {
        //Serial.println("No ack!");
        return (false); //Device failed to ack
    }

    return (true);
}
