Modifications
=============

## Compiler

Tested with toolchain-xtensa 2.100200.0 (10.2.0)

## Reset maximum power limit

To remove the power limit, set it to 0. It will not use any CPU cycles compared to setting an arbitrary high value.

```c++
FastLED.setMaxPowerInMilliWatts(0);
```

## Change power consumption per LED color

The amount is set in milliwatts per color for 256 LEDs turned to full brightness. The idle amount is measured with 0% brightness and added to each color.

The power was measured with an INA219 current sensor at the output of the buck regulator and there are no losses expect for the wiring.
The panel was a flexible 16x16 LED matrix with WS2812B, a decent amount of copper traces and 3 evenly spread connections to 5V.

The additional 125mW that has been added for the CPU has been removed.

```c++
inline void setPowerConsumptionInMilliwattsPer256(uint16_t red_mW, uint16_t green_mW, uint16_t blue_mW, uint16_t idle_mW);

// red   = 20418mW / 256 / 5V = 15.9mA + idle
// green = 20470mW / 256 / 5V = 15.99mA + idle
// blue  = 20379mW / 256 / 5V = 15.92mA + idle
// idle  = 1039mW / 256 / 5V = 0.81mA

FastLED.setPowerConsumptionInMilliwattsPer256(20418, 20470, 20379, 1039);
```

These numbers are pretty close the datasheet, which specifies 16.5/17.5mA (WS2813/WS2812B) per color at full brightness.

**NOTE**: When testing your LEDs, make sure your power supply can handle the transients. I have seen voltage spikes up to 8-9V on the 5V rail switching from 7A to 0A. Ramping down the power slowly within 50-100ms will avoid these issues.
If such sudden changes are required, a snubber network and/or a zener diode that can handle the overvoltage will protect the LEDs and other devices on the 5V rail.


## Measurinng the power

To measure the power, the function to calculate the power can be set and call the original function to intercept the results.

```c++
FastLED.setMaxPowerInMilliWatts(_config.power_limit * 1000, &calcPowerFunction);

uint32_t _powerLevelCurrentmW;

uint8_t _calcPowerFunction(uint8_t targetBrightness, uint32_t maxPower_mW)
{
	uint32_t requestedPower_mW;
	// call original function to calculate the power
	uint8_t newBrightness = _calculate_max_brightness_for_power_mW(targetBrightness, maxPower_mW, requestedPower_mW);
	// if the returned brightness does not match targetBrightness, maxPower_mW is used, otherwise requestedPower_mW
	_powerLevelCurrentmW = (targetBrightness == newBrightness) ? requestedPower_mW : maxPower_mW;
	_calcAveragePowerLevel();
	return newBrightness;
}

void _calcAveragePowerLevel()
{
	// integrate _powerLevelCurrentmW into average value...
}
```

## Working implementation with more features


A full implementation of the new features can be found at the Clock/LED matrix plugin of the KFC firmware

https://github.com/sascha432/esp8266-kfc-fw
https://github.com/sascha432/esp8266-kfc-fw/blob/master/src/plugins/clock/seven_segment_display.h
https://github.com/sascha432/esp8266-kfc-fw/tree/master/src/plugins/clock



[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/FastLED/public)
[![arduino-library-badge](https://www.ardu-badge.com/badge/FastLED.svg)](https://www.ardu-badge.com/FastLED)
![build status](https://github.com/FastLED/FastLED/workflows/build/badge.svg)

IMPORTANT NOTE: For AVR based systems, avr-gcc 4.8.x is supported and tested.  This means Arduino 1.6.5 and later.


FastLED 3.4
===========

This is a library for easily & efficiently controlling a wide variety of LED chipsets, like the ones
sold by adafruit (Neopixel, DotStar, LPD8806), Sparkfun (WS2801), and aliexpress.  In addition to writing to the
leds, this library also includes a number of functions for high-performing 8bit math for manipulating
your RGB values, as well as low level classes for abstracting out access to pins and SPI hardware, while
still keeping things as fast as possible. Tested with Arduino up to 1.6.5 from arduino.cc.

Quick note for people installing from GitHub repo zips, rename the folder FastLED before copying it to your Arduino/libraries folder.  Github likes putting -branchname into the name of the folder, which unfortunately, makes Arduino cranky!

We have multiple goals with this library:

* Quick start for new developers - hook up your leds and go, no need to think about specifics of the led chipsets being used
* Zero pain switching LED chipsets - you get some new leds that the library supports, just change the definition of LEDs you're using, et. voila!  Your code is running with the new leds.
* High performance - with features like zero cost global brightness scaling, high performance 8-bit math for RGB manipulation, and some of the fastest bit-bang'd SPI support around, FastLED wants to keep as many CPU cycles available for your led patterns as possible

## Getting help

If you need help with using the library, please consider going to the reddit community first, which is at http://fastled.io/r (or https://reddit.com/r/FastLED) - there are thousands of great people in that group and many times you will get a quicker answer to your question there, as you will be likely to run into other people who have had the same issue.  If you run into bugs with the library (compilation failures, the library doing the wrong thing), or if you'd like to request that we support a particular platform or LED chipset, then please open an issue at http://fastled.io/issues and we will try to figure out what is going wrong.

## Simple example

How quickly can you get up and running with the library?  Here's a simple blink program:

	#include "FastLED.h"
	#define NUM_LEDS 60
	CRGB leds[NUM_LEDS];
	void setup() { FastLED.addLeds<NEOPIXEL, 6>(leds, NUM_LEDS); }
	void loop() {
		leds[0] = CRGB::White; FastLED.show(); delay(30);
		leds[0] = CRGB::Black; FastLED.show(); delay(30);
	}

## Supported LED chipsets

Here's a list of all the LED chipsets are supported.  More details on the led chipsets are included *TODO: Link to wiki page*

* Adafruit's DotStars - AKA the APA102
* Adafruit's Neopixel - aka the WS2812B (also WS2811/WS2812/WS2813, also supported in lo-speed mode) - a 3 wire addressable led chipset
* TM1809/4 - 3 wire chipset, cheaply available on aliexpress.com
* TM1803 - 3 wire chipset, sold by radio shack
* UCS1903 - another 3 wire led chipset, cheap
* GW6205 - another 3 wire led chipset
* LPD8806 - SPI based chipset, very high speed
* WS2801 - SPI based chipset, cheap and widely available
* SM16716 - SPI based chipset
* APA102 - SPI based chipset
* P9813 - aka Cool Neon's Total Control Lighting
* DMX - send rgb data out over DMX using arduino DMX libraries
* SmartMatrix panels - needs the SmartMatrix library - https://github.com/pixelmatix/SmartMatrix
* LPD6803 - SPI based chpiset, chip CMODE pin must be set to 1 (inside oscillator mode)


HL1606, and "595"-style shift registers are no longer supported by the library.  The older Version 1 of the library ("FastSPI_LED") has support for these, but is missing many of the advanced features of current versions and is no longer being maintained.


## Supported platforms

Right now the library is supported on a variety of arduino compatable platforms.  If it's ARM or AVR and uses the arduino software (or a modified version of it to build) then it is likely supported.  Note that we have a long list of upcoming platforms to support, so if you don't see what you're looking for here, ask, it may be on the roadmap (or may already be supported).  N.B. at the moment we are only supporting the stock compilers that ship with the arduino software.  Support for upgraded compilers, as well as using AVR studio and skipping the arduino entirely, should be coming in a near future release.

* Arduino & compatibles - straight up arduino devices, uno, duo, leonardo, mega, nano, etc...
* Arduino Yún
* Adafruit Trinket & Gemma - Trinket Pro may be supported, but haven't tested to confirm yet
* Teensy 2, Teensy++ 2, Teensy 3.0, Teensy 3.1/3.2, Teensy LC, Teensy 3.5, Teensy 3.6, and Teensy 4.0 - arduino compataible from pjrc.com with some extra goodies (note the teensy 3, 3.1, and LC are ARM, not AVR!)
* Arduino Due and the digistump DigiX
* RFDuino
* SparkCore
* Arduino Zero
* ESP8266 using the arduino board definitions from http://arduino.esp8266.com/stable/package_esp8266com_index.json - please be sure to also read https://github.com/FastLED/FastLED/wiki/ESP8266-notes for information specific to the 8266.
* The wino board - http://wino-board.com
* ESP32 based boards

What types of platforms are we thinking about supporting in the future?  Here's a short list:  ChipKit32, Maple, Beagleboard

## What about that name?

Wait, what happend to FastSPI_LED and FastSPI_LED2?  The library was initially named FastSPI_LED because it was focused on very fast and efficient SPI access.  However, since then, the library has expanded to support a number of LED chipsets that don't use SPI, as well as a number of math and utility functions for LED processing across the board.  We decided that the name FastLED more accurately represents the totality of what the library provides, everything fast, for LEDs.

## For more information

Check out the official site http://fastled.io for links to documentation, issues, and news


*TODO* - get candy
