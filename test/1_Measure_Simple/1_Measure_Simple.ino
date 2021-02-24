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
  Test_AS726XX.ino
  Simple test code for AS726XX-CommonLib.
  Examined for AS7265X, AS7262, AS7341.
  Kiyo Chinzei
  https://github.com/kchinzei/AS726XX-CommonLib
*/

#include "AS726XX.h"

AS726XX sensor;

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Serial.println("AS7262/7263/7265X Example:");
  Serial.println("This example work also for AS7341, but measurement returns 0, because it doesn't have getA()/getCalibratedA() etc.");

  if(sensor.begin() == false)
  {
    Serial.println("Sensor does not appear to be connected. Please check wiring. Freezing...");
    while(1);
  }

  sensor.setIntegrationCycles(100);
  sensor.setGain(AS7265X_GAIN_64X);
  
  byte deviceType = sensor.getDeviceType();
  Serial.print("AMS Device Type: 0x");
  Serial.println(deviceType, HEX);

  byte hardwareVersion = sensor.getHardwareVersion();
  Serial.print("AMS Hardware Version: 0x");
  Serial.println(hardwareVersion, HEX);

  byte majorFirmwareVersion = sensor.getMajorFirmwareVersion();
  Serial.print("Major Firmware Version: 0x");
  Serial.println(majorFirmwareVersion, HEX);

  byte patchFirmwareVersion = sensor.getPatchFirmwareVersion();
  Serial.print("Patch Firmware Version: 0x");
  Serial.println(patchFirmwareVersion, HEX);

  byte buildFirmwareVersion = sensor.getBuildFirmwareVersion();
  Serial.print("Build Firmware Version: 0x");
  Serial.println(buildFirmwareVersion, HEX);

  Serial.print("UV Bulb: ");
  Serial.println(sensor.isUVBulbAvailable()? "Yes":"No");
  Serial.print("White Bulb: ");
  Serial.println(sensor.isWhiteBulbAvailable()? "Yes":"No");
  Serial.print("IR Bulb: ");
  Serial.println(sensor.isIRBulbAvailable()? "Yes":"No");

  uint16_t v;
  if (v = sensor.getAnm()) {
    Serial.print("A[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getBnm()) {
    Serial.print("B[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getCnm()) {
    Serial.print("C[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getDnm()) {
    Serial.print("D[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getEnm()) {
    Serial.print("E[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getFnm()) {
    Serial.print("F[");
    Serial.print(v);
    Serial.print("nm] ");
  }

  if (v = sensor.getGnm()) {
    Serial.print("G[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getHnm()) {
    Serial.print("H[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getInm()) {
    Serial.print("I[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getJnm()) {
    Serial.print("J[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getKnm()) {
    Serial.print("K[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getLnm()) {
    Serial.print("L[");
    Serial.print(v);
    Serial.print("nm] ");
  }

  if (v = sensor.getRnm()) {
    Serial.print("R[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getSnm()) {
    Serial.print("S[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getTnm()) {
    Serial.print("T[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getUnm()) {
    Serial.print("U[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getVnm()) {
    Serial.print("V[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  if (v = sensor.getWnm()) {
    Serial.print("W[");
    Serial.print(v);
    Serial.print("nm] ");
  }
  Serial.println("");
}

void loop() {
  sensor.disableIndicator();
  sensor.takeMeasurements(); //This is a hard wait while all channels are measured
  // sensor.takeMeasurementsWithBulb();

  if (sensor.getAnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedA(), 1);
    Serial.print(" ");
  }
  if (sensor.getBnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedB(), 1);
    Serial.print(" ");
  }
  if (sensor.getCnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedC(), 1);
    Serial.print(" ");
  }
  if (sensor.getDnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedD(), 1);
    Serial.print(" ");
  }
  if (sensor.getEnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedE(), 1);
    Serial.print(" ");
  }
  if (sensor.getFnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedF(), 1);
    Serial.print(" ");
  }

  if (sensor.getGnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedG(), 1);
    Serial.print(" ");
  }
  if (sensor.getHnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedH(), 1);
    Serial.print(" ");
  }
  if (sensor.getInm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedI(), 1);
    Serial.print(" ");
  }
  if (sensor.getJnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedJ(), 1);
    Serial.print(" ");
  }
  if (sensor.getKnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedK(), 1);
    Serial.print(" ");
  }
  if (sensor.getLnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedL(), 1);
    Serial.print(" ");
  }

  if (sensor.getRnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedR(), 1);
    Serial.print(" ");
  }
  if (sensor.getSnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedS(), 1);
    Serial.print(" ");
  }
  if (sensor.getTnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedT(), 1);
    Serial.print(" ");
  }
  if (sensor.getUnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedU(), 1);
    Serial.print(" ");
  }
  if (sensor.getVnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedV(), 1);
    Serial.print(" ");
  }
  if (sensor.getWnm()) {
    Serial.print("  ");
    Serial.print(sensor.getCalibratedW(), 1);
    Serial.print(" ");
  }
  Serial.println("");
  sensor.enableIndicator();
}
