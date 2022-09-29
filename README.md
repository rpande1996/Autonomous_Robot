## Autonomous_Robot
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This code demonstrated autonomous run of a Raspberry Pi robot (Barron) using direction detection, obstacle detection, face recognition and a qr code scanner

## Hardware

* Raspberry Pi 3- Model B+ (1.4 GHz Cortex-A53 with 1GB RAM) x1
[Buy](https://www.adafruit.com/product/3775)

* Raspberry Pi Camera Board v2 (8 MP) x1
[Buy](https://www.adafruit.com/product/3099)

* 16 GB Memory card for Raspberry Pi x1
[Buy](https://www.adafruit.com/product/4266)

* Half-sized breadboard x1
[Buy](https://www.adafruit.com/product/64)

* Ultrasonic Distance sensor x1
[Buy](https://www.adafruit.com/product/4007)

* 5V 2.5A micro-usb power supply x1
[Buy](https://www.adafruit.com/product/1995)

* 9-DOF IMU (BNO055) x1
[Buy](https://www.adafruit.com/product/2472)

* USB A to Micro-USB cable (3 ft) x1
[Buy](https://www.adafruit.com/product/592)

* DFRobot 4WD Arduino compatible platform w/ encoders (Barror)
[Buy](https://www.robotshop.com/en/dfrobot-4wd-arduino-platform-encoders.html?utm_source=google&utm_medium=surfaces&utm_campaign=surfaces_across_google_usen&gclid=CjwKCAjw_NX7BRA1EiwA2dpg0tB3INHXEuIw4m0F4IL5-xNskpYiofWkfy6RqS66eA5lRMDNr84NzxoCpCYQAvD_BwE)

* 10000mAh Power bank x1
[Buy](https://www.amazon.com/gp/product/B07G26S5V8)

* 12in 40 pin male to female dupont wire x1
[Buy](https://www.amazon.com/gp/product/B06XRV92ZB)

* Motor drive controller board module (H-Bridge) x1
[Buy](https://www.amazon.com/gp/product/B07C4B3DL4)

* Rechargeable battery charger x1
[Buy](https://www.amazon.com/gp/product/B00IM3P8GS)

* Rechargeable batteries x1
[Buy](https://www.amazon.com/gp/product/B00HZV9WTM)

* Hitec Servo Motor x1
[Buy](https://www.amazon.com/gp/product/B0006O3WVE/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)

* Arduino Nano x1
[Buy](https://www.amazon.com/Arduino-A000005-ARDUINO-Nano/dp/B0097AU5OU/ref=redir_mobile_desktop?_encoding=UTF8&aaxitk=RSNsxDkXeAsnRpCC1AKKWw&hsa_cr_id=9484023550601)

* 1ft USB-A to Mini-B x1
[Buy](https://www.amazon.com/Antrader-1-Feet-Cable-Mini-B-Length/dp/B07DYFN1ZQ/ref=sr_1_6?keywords=USB%2BA%2FminiB%2B1%2Bfoot&qid=1581267524&sr=8-6&th=1)

* Parallel Gripper Kit x1
[Buy](https://www.servocity.com/parallel-gripper-kit-a)

## Libraries

* OpenCV 3.4.8.29
* Numpy 1.21.2
* Matplotlib 3.3.4

## Programming Languages

* Python 3.7

## License 

```
MIT License

Copyright (c) 2021 Rajan Pande

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
```

## Demo

Autonomous Run:

Run 1:

![Run1](media/output/output1.gif)

Run 2:

![Run2](media/output/output2.gif)


Paths for the respective runs:

Path 1:

![Path1](media/output/path2.png)

Path 2:

![Path2](media/output/path1.png)

## Build

```
git clone https://github.com/rpande1996/Autonomous_Robot
cd Autonomous_Robot/src
python main.py
```
