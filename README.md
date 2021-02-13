# AS726XX-CommonLib

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Wrapper class for [SparkFun AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi) and [SparkFun AS7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) to hide differences between them.

## About SparkFun [AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi)/[7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide)

[AS7262/7263](https://learn.sparkfun.com/tutorials/as726x-nirvi)/[7265x](https://learn.sparkfun.com/tutorials/spectral-triad-as7265x-hookup-guide) are spectral sensors for visual light, NIR, and UV-NIR bands.

Arduino library for [AS7262/7263](https://github.com/sparkfun/Sparkfun_AS726X_Arduino_Library) and [AS7265x](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library) are similar, but there are differences that hinder writing a single code to use these sensors.

![AS7265x](https://cdn.sparkfun.com/r/500-500/assets/parts/1/3/3/9/3/15050-SparkFun_Triad_Spectroscopy_Sensor_-_AS7265x__Qwiic_-01.jpg "Overview of AS7265x")

## H/W differences
AS726XX-CommonLib does not hide differences in hardware.

- Available spectrum bands: Peak frequency of spectrum bands do overwrap. However, frequencies are different.
- LED bulbs: There are 3 LEDs on AS7265x while only one on AS7262/7263.

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
