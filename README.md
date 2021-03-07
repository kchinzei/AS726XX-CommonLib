# AS726XX-CommonLib

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Wrapper class for [SparkFun AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi), [Adafruit AS7341](https://learn.adafruit.com/adafruit-as7341-10-channel-light-color-sensor-breakout) and [SparkFun AS7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) to hide differences between them. Written as Arduino library, but it should work with PlatformIO.

## About [AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi)/[7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) and [AS7341](https://learn.adafruit.com/adafruit-as7341-10-channel-light-color-sensor-breakout)

[AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi)/[7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) and [AS7341](https://learn.adafruit.com/adafruit-as7341-10-channel-light-color-sensor-breakout) are breakout boards from Sparkfun and Adafruit with spectral sensors for visual light, NIR, and UV-NIR bands. Basically, all these use AMS Chips therefore basic architecture is common and alike.

Arduino library classes and functions for [AS7262/7263](https://github.com/sparkfun/Sparkfun_AS726X_Arduino_Library) and [AS7265x](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library) from Sparkfun are similar, but they are slightly different. That of [AS7341](https://github.com/adafruit/Adafruit_AS7341) from Adafruit is certainly different. This means that you must write different codes to use these sensors. By using `AS726XX` class, your code can treat AS7262/AS7263 and AS7341 as if AS7265x is connected - single code works for all. (But of course, it's not capable to mimic missing hardware.)


![AS7265x](https://cdn.sparkfun.com/r/500-500/assets/parts/1/3/3/9/3/15050-SparkFun_Triad_Spectroscopy_Sensor_-_AS7265x__Qwiic_-01.jpg "View of AS7265x")
![AS7341](https://cdn-learn.adafruit.com/guides/cropped_images/000/003/067/medium640/4698_top_ORIG_2020_08.jpg?1597258397 "View of S7341")

## Using AS726XX-CommonLib

Copy AS726XX.cpp and AS726XX.h to where you want to use. Or put them in a folder under Arduino's 'libraries' folder.

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

You can find examples in [test](https://github.com/kchinzei/AS726XX-CommonLib/tree/as7341/test) and my [ESP8266-AS7265x-Server](https://github.com/kchinzei/ESP8266-AS7265x-Server).

### Array of available channels

`AS726XX` provides arrays to access available spectrum channels.
Instead of using `getCalibratedA()` etc, you can do like:

```C++
AS726XX sensor;
  ...
for (int i; i<sensor.maxCh; i++) {
   float val = sensor.readings[i];ÂÂ
   ...
```

Available arrays are:
- `uint16_t AS726XX::nm;` : Array of wavelengths in \[nm\].
- `float AS726XX::readings;` : Array of obtained values.
- `float AS726XX::cal_params;` : Array of calibration parameters for calibration, multiplied to `getCalibratedA()` to `getCalibaratedW()` for AS7265x and `readAllChannels()` for AS7341.

Array length is in `AS726XX::maxCh`. These are not intended to user modifies its contents except `cal_params`.

### Compile without AS7262/7263 library

If you don't need and want AS7262/7263 now, you can compile AS726XX-CommonLib without [AS7262/7263](https://github.com/sparkfun/Sparkfun_AS726X_Arduino_Library) library.
When doing so, comment out `#include <AS726X.h>` in AS726XX.h

Similarly if you don't use Adafruit AS7341, comment out `#include <Adafruit_AS7341.h>` in AS726XX.h

(If `__has_include()` works, it could automatically switch without manual modification - this preprocessor of Arduino IDE does not correctly work currently.)

### H/W differences

An obvious difference is that AS7265x has 3 sensor chips while the others have only one. `AS726XX` behaves as the following.

- **Spectrum bands:** Some peak frequency of spectrum bands do overwrap. However, the wavelength are different. You can examine the band's wavelength and if the band is available by using `uint16_t getAnm()` etc. Obtaining not-existing band spectrum returns 0.
- **LED bulbs:** There are 3 LEDs on AS7265x while only one on AS7262/7263. You can examine if specific LED is available by using `boolean isUVBulbAvailable()` etc. Attempting to enable or disable not-existing LED is silently ignored.
- **Temperature:** `uint8_t getTemperature(uint8_t deviceNumber)` provides temperature of each chip on AS7265x specified by `deviceNumber`. `float getTemperatureAverage()` gives the average of them. AS7262/7263 has only one chip, so it always returns the temperature of this chip regardless to `deviceNumber`.
- **Acquisition time:** AS7265x takes more time to obtain spectrum value by using `getCalibratedA()` etc. However, integration time itself are almost same as AS7262/7263.

AS7341 is a new IC and many minor differences from AS726x/AS7265x series.
- **Indicator always on**. Not programmable.
- **Much faster sampling cycle**. It's in microsec order while AS726x/AS7265x are in millisec orders.
- **Gain is between 1/2 and 512**. AS7265x/726x is between 1 and 64. If you provide the gain larger than `AS7265X_GAIN_64X` (=0x03) to AS7265x/726x, it cuts off to `AS7265X_GAIN_64X`.
- Many interrupt modes.

### Internal structure

Inside AS726XX-CommonLib manages a pointer to the original `AS7265X` or `AS726X` object depending on the detected hardware. It would be nicely rewritten using an abstract class. However doing so requires a lot of rewriting of the SparkFun's original codes. The current approach is not C++ like, but it will absorb future changes in SparkFun codes [when they fix bugs in their codes](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library/issues/11).

##### Why not a child class of `AS7265X`?
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
