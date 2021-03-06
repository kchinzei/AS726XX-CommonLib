# AS726XX-CommonLib

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Wrapper class for [SparkFun AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi) and [SparkFun AS7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) to hide differences between them. Written as Arduino library, but it should work with PlatformIO.

## About SparkFun [AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi)/[7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide)

[AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi)/[7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) are breakout boards with spectral sensors for visual light, NIR, and UV-NIR bands.

Arduino library classes and functions for [AS7262/7263](https://github.com/sparkfun/Sparkfun_AS726X_Arduino_Library) and [AS7265x](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library) are similar, but they are slightly different. So you must write slightly different code code to use these sensors. By using `AS726XX` class, your code can treat AS7262/AS7263 as if AS7265x is connected - single code works for all.


![AS7265x](https://cdn.sparkfun.com/r/500-500/assets/parts/1/3/3/9/3/15050-SparkFun_Triad_Spectroscopy_Sensor_-_AS7265x__Qwiic_-01.jpg "Overview of AS7265x")

## Using AS726XX-CommonLib

Copy AS726XX.cpp and AS726XX.h to where you want to use. Or put the folder into 'libraries' folder where Arduino can search for files.

You instantiate an `AS726XX` object. It has the interface same as that of [AS7265x library](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library). Therefore, you can interchangeably use `AS726XX` object where `AS7265X` object appears.

```C++
#include "AS726XX.h"
AS726XX sensor;

void setup() {
  if(sensor.begin() == false) {
    Serial.println("Sensor does not appear to be connected. Freezing...");
    while(1);
  }
```

You can find examples in [Test_AS726XX.ino](https://github.com/kchinzei/AS726XX-CommonLib/blob/master/Test_AS726XX/Test_AS726XX.ino) and my [ESP8266-AS7265x-Server](https://github.com/kchinzei/ESP8266-AS7265x-Server).

### Vectors of available channels

`AS726XX` provides vectors to access available spectrum channels.
Instead of using `getCalibratedA()` etc, you can do like:

```C++
AS726XX sensor;
  ...
for (auto itr=std::begin(sensor.getc); itr != std::end(sensor.getc); itr++) {
   float val = (sensor.*(*itr))();
   ...
```

Available vectors are:
- `std::vector<uint16_t> nm;` : Array of wavelengths in \[nm\].
- `std::vector<uint16_t_fptr> getu;` : Array of class functions to get uncalibrated values such as `getA()`.
- `std::vector<float_fptr> getc;` : Array of class functions to get calibrated values such as `getCalibratedA()`.

These are defined as mutable array, but it is not intended to user modifies its contents. Doing so will result erroneous behavior.

### Compile without AS7262/7263 library

If you don't need and want AS7262/7263 now, you can compile AS726XX-CommonLib without [AS7262/7263](https://github.com/sparkfun/Sparkfun_AS726X_Arduino_Library) library.
When doing so, comment out `#include <AS726X.h>` in AS726XX.h

### H/W differences

An obvious difference is that AS7265x has 3 sensor chips while AS7262/7263 has only one. `AS726XX` behaves as the following.

- **Spectrum bands:** Some peak frequency of spectrum bands do overwrap. However, the wavelength are different. You can examine the band's wavelength and if the band is available by using `uint16_t getAnm()` etc. Obtaining not-existing band spectrum returns 0.
- **LED bulbs:** There are 3 LEDs on AS7265x while only one on AS7262/7263. You can examine if specific LED is available by using `boolean isUVBulbAvailable()` etc. Attempting to enable or disable not-existing LED is silently ignored.
- **Temperature:** `uint8_t getTemperature(uint8_t deviceNumber)` provides temperature of each chip on AS7265x specified by `deviceNumber`. `float getTemperatureAverage()` gives the average of them. AS7262/7263 has only one chip, so it always returns the temperature of this chip regardless to `deviceNumber`.
- **Acquisition time:** AS7265x takes more time to obtain spectrum value by using `getCalibratedA()` etc. However, integration time itself are almost same as AS7262/7263.

### Internal structure

Inside AS726XX-CommonLib manages a pointer to the original `AS7265X` or `AS726X` object depending on the detected hardware. It would be nicely rewritten using an abstract class. However doing so requires a lot of rewriting of the SparkFun's original codes. The current approach is not C++ like, but it will absorb future changes in SparkFun codes [when they fix bugs in their codes](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library/issues/11).

Why not a child class of `AS7265X`?
You can also write a class `AS726XX` as a child class of `AS7265X`.
By doing so you can assure and inherit the interface of `AS7265X`.
When you connect AS7262/AS7263, you manage an `AS726X` object separately.
Simple. But that case allocates unused `AS7265X` object.
I wanted memory footprint as small as possible.

## Note

- Using AS7262, AS7263, AS7265x in the same I2C bus requires [Qwiic MUX board](https://learn.sparkfun.com/tutorials/qwiic-mux-hookup-guide) since they have the same I2C address.
- I don't have AS7263, means I did not tested for it.

# License

The MIT License (MIT)
Copyright (c) K. Chinzei (kchinzei@gmail.com)
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
