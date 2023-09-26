#define FASTLED_INTERNAL
#include "FastLED.h"
#include "power_mgt.h"

FASTLED_NAMESPACE_BEGIN

//// POWER MANAGEMENT

// These power usage values are approximate, and your exact readings
// will be slightly (10%?) different from these.
//
// They were arrived at by actually measuring the power draw of a number
// of different LED strips, and a bunch of closed-loop-feedback testing
// to make sure that if we USE these values, we stay at or under
// the target power consumption.
// Actual power consumption is much, much more complicated and has
// to include things like voltage drop, etc., etc.
// However, this is good enough for most cases, and almost certainly better
// than no power management at all.
//
// You're welcome to adjust these values as needed; there may eventually be an API
// for changing these on the fly, but it saves codes pace and RAM to have them
// be compile-time constants.


// the values have been measured with a WS2812B 16x16 LED panel at full brightness

// scale all power values by this factor
constexpr float kPowerScale = 1.0;
// 256 LEDs, power in milliwatt for each color at 100%
static uint16_t gRed_mW   = 20418 * kPowerScale; // 20864mW / 256 / 5V = 16.3mA
static uint16_t gGreen_mW = 20470 * kPowerScale; // 21120mW / 256 / 5V = 16.4mA
static uint16_t gBlue_mW  = 20379 * kPowerScale; // 20736mW / 256 / 5V = 16.2mA
// all 256 LEDs at 0%
static uint16_t gDark_mW  = 1039 * kPowerScale; // 1039mW / 256 / 5V = 0.81mA

// set power consumption in milliwatt per color for 256 LEDs
void set_power_consumption(uint16_t red, uint16_t green, uint16_t blue, uint16_t idle)
{
    gRed_mW = red;
    gGreen_mW = green;
    gBlue_mW = blue;
    gDark_mW = idle;
}

// Power consumed by the MCU
#ifndef POWER_MCU_MW
#define POWER_MCU_MW 0
#endif

inline static uint32_t calc_per_color(uint32_t value, uint16_t mW_for256)
{
    // value can get up to 24bit, since we have 8bit precision
    // check if we have more than 16bit and reduce to 16bit if necessary
    return (((value >= 0xffff) ? (value >> 8) * mW_for256 : (value * mW_for256) >> 8) + 1) >> 8;
}

uint32_t calculate_idle_power_mW(uint16_t numLeds)
{
    return calc_per_color(numLeds, gDark_mW);
}

uint32_t calculate_idle_power_mW()
{
    uint32_t total_mW = 0;
    CLEDController *pCur = CLEDController::head();
	while(pCur) {
        total_mW += calculate_idle_power_mW(pCur->size());
		pCur = pCur->next();
	}
    return total_mW + POWER_MCU_MW;
}

uint32_t calculate_unscaled_power_mW(const CRGB *ledbuffer, uint16_t numLeds)
{
    uint32_t red32 = 0, green32 = 0, blue32 = 0;
    auto p = reinterpret_cast<const uint8_t *>(&ledbuffer[0]);

    // This loop might benefit from an AVR assembly version -MEK
    while(numLeds--) {
        red32   += *p++;
        green32 += *p++;
        blue32  += *p++;
    }

    // calculate total power at 100% brightness
    return calc_per_color(red32, gRed_mW) + calc_per_color(green32, gGreen_mW) + calc_per_color(blue32, gBlue_mW);
}

uint8_t calculate_max_brightness_for_power_vmA(const CRGB *ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_V, uint32_t max_power_mA)
{
	return calculate_max_brightness_for_power_mW(ledbuffer, numLeds, target_brightness, max_power_V * max_power_mA);
}

inline static uint8_t _get_brightness(uint8_t target_brightness, uint32_t max_power_mW, uint32_t total_mW, uint32_t &requested_power_mW)
{
	requested_power_mW = ((total_mW * (target_brightness + 1)) >> 8) + POWER_MCU_MW;
    if (requested_power_mW <= max_power_mW) {
	    return target_brightness;
	}
    return ((max_power_mW * target_brightness) + POWER_MCU_MW) / requested_power_mW;
}

uint8_t calculate_max_brightness_for_power_mW(const CRGB* ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_mW)
{
 	uint32_t total_mW = calculate_unscaled_power_mW(ledbuffer, numLeds);
    uint32_t requested_power_mW;
    return _get_brightness(target_brightness, max_power_mW, total_mW, requested_power_mW);
}

uint8_t _calculate_max_brightness_for_power_mW(uint8_t target_brightness, uint32_t max_power_mW, uint32_t &requested_power_mW)
{
    uint32_t total_mW = 0;
    CLEDController *pCur = CLEDController::head();
	while(pCur) {
        total_mW += calculate_unscaled_power_mW(pCur->leds(), pCur->size());
		pCur = pCur->next();
	}
    return _get_brightness(target_brightness, max_power_mW, total_mW, requested_power_mW);
}

void set_max_power_in_volts_and_milliamps(uint8_t volts, uint32_t milliamps)
{
    FastLED.setMaxPowerInVoltsAndMilliamps(volts, milliamps);
}

void set_max_power_in_milliwatts(uint32_t powerInmW)
{
    FastLED.setMaxPowerInMilliWatts(powerInmW);
}

FASTLED_NAMESPACE_END
