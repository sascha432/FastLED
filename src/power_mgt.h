#ifndef POWER_MGT_H
#define POWER_MGT_H

#include "FastLED.h"

#include "pixeltypes.h"

FASTLED_NAMESPACE_BEGIN

///@defgroup Power Power management functions
/// functions used to limit the amount of power used by FastLED
///@{

// Power Control setup functions
//
// Example:
//  set_max_power_in_volts_and_milliamps( 5, 400);
//

/// Set the maximum power used in milliamps for a given voltage
/// @deprecated - use FastLED.setMaxPowerInVoltsAndMilliamps()
void set_max_power_in_volts_and_milliamps( uint8_t volts, uint32_t milliamps);
/// Set the maximum power used in watts
/// @deprecated - use FastLED.setMaxPowerInMilliWatts
void set_max_power_in_milliwatts( uint32_t powerInmW);

// Power Control internal helper functions

/// calculate_unscaled_power_mW tells you how many milliwatts the current
///   LED data would draw at brightness = 255.
///
uint32_t calculate_unscaled_power_mW( const CRGB* ledbuffer, uint16_t numLeds);

// get power for 0 brightness
uint32_t calculate_idle_power_mW(uint16_t numLeds);

// get idle power for all strips
uint32_t calculate_idle_power_mW();

/// calculate_max_brightness_for_power_mW tells you the highest brightness
///   level you can use and still stay under the specified power budget for
///   a given set of leds.  It takes a pointer to an array of CRGB objects, a
///   count, a 'target brightness' which is the brightness you'd ideally like
///   to use, and the max power draw desired in milliwatts.  The result from
///   this function will be no higher than the target_brightess you supply, but may be lower.
uint8_t calculate_max_brightness_for_power_mW(const CRGB* ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_mW);

/// calculate_max_brightness_for_power_mW tells you the highest brightness
///   level you can use and still stay under the specified power budget for
///   a given set of leds.  It takes a pointer to an array of CRGB objects, a
///   count, a 'target brightness' which is the brightness you'd ideally like
///   to use, and the max power in volts and milliamps.  The result from this
///   function will be no higher than the target_brightess you supply, but may be lower.
uint8_t calculate_max_brightness_for_power_vmA(const CRGB* ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_V, uint32_t max_power_mA);

/// calculate_max_brightness_for_power_mW tells you the highest brightness
///   level you can use and still stay under the specified power budget.  It
///   takes a 'target brightness' which is the brightness you'd ideally like
///   to use.  The result from this function will be no higher than the
///   target_brightess you supply, but may be lower.
///   requested_power_mW is set to total_power if the brightness is not reduced
uint8_t _calculate_max_brightness_for_power_mW(uint8_t target_brightness, uint32_t max_power_mW, uint32_t &requested_power_mW);

inline static uint8_t calculate_max_brightness_for_power_mW(uint8_t target_brightness, uint32_t max_power_mW)
{
    uint32_t requested_power_mW;
    return _calculate_max_brightness_for_power_mW(target_brightness, max_power_mW, requested_power_mW);
}

// how to measure the power:
//
// idle power = measured voltage * idle current(mA) * 256 / number of LEDs tested
// color power = (measured voltage * color current(mA) * 256 / number of LEDs tested) - idle power
//
//  the idle power:
//   turn all LEDs off and measure current and voltage
//  repeat this for each color
//   turn brightness to 100% and measure current and voltage
//   subtract the idle power from each value

// set power consumption in milliwatt per color for 256 LEDs
void set_power_consumption(uint16_t red_mW, uint16_t green_mW, uint16_t blue_mW, uint16_t idle_mW);

// set power consumption in milliwatt per color for 256 LEDs
inline void set_power_consumption_mW(uint16_t red_mW, uint16_t green_mW, uint16_t blue_mW, uint16_t idle_mW)
{
    set_power_consumption(red_mW, green_mW, blue_mW, idle_mW);
}

// set power consumption in milliamps per color for a single LED
// the default voltage is 5.0. use set_power_consumption_mW() if the voltage is different
void set_power_consumption_mA(float red_mA, float green_mA, float blue_mA, float idle_mA);

FASTLED_NAMESPACE_END
///@}
// POWER_MGT_H

#endif
