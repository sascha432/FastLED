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


// the values have been measured with a 16x16 LED panel at full brightness
constexpr float powerScale = 1.0;
// 256 LEDs, power in milliwatt for each color at 100%
static uint16_t gRed_mW   = 20418 / powerScale; // 20864mW / 256 / 5V = 16.3mA
static uint16_t gGreen_mW = 20470 / powerScale; // 21120mW / 256 / 5V = 16.4mA
static uint16_t gBlue_mW  = 20379 / powerScale; // 20736mW / 256 / 5V = 16.2mA
// all 256 LEDs at 0%
static uint16_t gDark_mW  = 1039 / powerScale; // 1039mW / 256 / 5V = 0.81mA

// set power consumption in milliwatt per color for 256 LEDs
void set_power_consumption(uint16_t red, uint16_t green, uint16_t blue, uint16_t idle)
{
    gRed_mW = red;
    gGreen_mW = green;
    gBlue_mW = blue;
    gDark_mW = idle;
}

#define POWER_LED 0
#define POWER_DEBUG_PRINT 0

// Power consumed by the MCU
#ifndef POWER_MCU_MW
#define POWER_MCU_MW 0
#endif

#if POWER_LED > 0
static uint8_t gMaxPowerIndicatorLEDPinNumber = 0; // default = Arduino onboard LED pin.  set to zero to skip this.
#endif

static inline uint32_t calc_per_color(uint32_t value, uint16_t mW_for256)
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

uint32_t calculate_unscaled_power_mW(const CRGB* ledbuffer, uint16_t numLeds)
{
    uint32_t red32 = 0, green32 = 0, blue32 = 0;
    auto firstled = &(ledbuffer[0]);
    auto p = (uint8_t *)(firstled);

    // This loop might benefit from an AVR assembly version -MEK
    while(numLeds--) {
        red32   += *p++;
        green32 += *p++;
        blue32  += *p++;
    }

    // calculate total power at 100% brightness
    return calc_per_color(red32, gRed_mW) + calc_per_color(green32, gGreen_mW) + calc_per_color(blue32, gBlue_mW);
}

uint8_t calculate_max_brightness_for_power_vmA(const CRGB* ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_V, uint32_t max_power_mA)
{
	return calculate_max_brightness_for_power_mW(ledbuffer, numLeds, target_brightness, max_power_V * max_power_mA);
}

uint8_t calculate_max_brightness_for_power_mW(const CRGB* ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_mW)
{
 	uint32_t total_mW = calculate_unscaled_power_mW(ledbuffer, numLeds);

	uint32_t requested_power_mW = ((total_mW * (target_brightness + 1)) >> 8) + POWER_MCU_MW;

	uint8_t recommended_brightness = target_brightness;
	if(requested_power_mW > max_power_mW) {
        recommended_brightness = ((max_power_mW * target_brightness) + POWER_MCU_MW) / requested_power_mW;
	}

	return recommended_brightness;
}

uint8_t _calculate_max_brightness_for_power_mW(uint8_t target_brightness, uint32_t max_power_mW, uint32_t &requested_power_mW)
{
    uint32_t total_mW = 0;

    CLEDController *pCur = CLEDController::head();
	while(pCur) {
        total_mW += calculate_unscaled_power_mW(pCur->leds(), pCur->size());
		pCur = pCur->next();
	}

#if POWER_DEBUG_PRINT == 1
    Serial.print("power demand at full brightness mW = ");
    Serial.println(total_mW);
#endif

    requested_power_mW = ((total_mW * (target_brightness + 1)) >> 8) + POWER_MCU_MW;
#if POWER_DEBUG_PRINT == 1
    if(target_brightness != 255) {
        Serial.print("power demand at scaled brightness mW = ");
        Serial.println(requested_power_mW);
    }
    Serial.print("power limit mW = ");
    Serial.println(max_power_mW);
#endif

    if(requested_power_mW <= max_power_mW) {
#if POWER_LED > 0
        if(gMaxPowerIndicatorLEDPinNumber) {
            Pin(gMaxPowerIndicatorLEDPinNumber).lo(); // turn the LED off
        }
#endif
#if POWER_DEBUG_PRINT == 1
        Serial.print("demand is under the limit");
#endif
        return target_brightness;
    }

    uint8_t recommended_brightness = ((max_power_mW * target_brightness) + POWER_MCU_MW) / requested_power_mW;
#if POWER_DEBUG_PRINT == 1
    Serial.print("recommended brightness # = ");
    Serial.println(recommended_brightness);

    uint32_t resultant_power_mW = ((total_mW * (recommended_brightness * 1)) / 256) + POWER_MCU_MW;
    Serial.print("resultant power demand mW = ");
    Serial.println(resultant_power_mW);

    Serial.println();
#endif

#if POWER_LED > 0
    if(gMaxPowerIndicatorLEDPinNumber) {
        Pin(gMaxPowerIndicatorLEDPinNumber).hi(); // turn the LED on
    }
#endif
    return recommended_brightness;
}

void set_max_power_indicator_LED(uint8_t pinNumber)
{
#if POWER_LED > 0
    gMaxPowerIndicatorLEDPinNumber = pinNumber;
#endif
}

void set_max_power_in_volts_and_milliamps(uint8_t volts, uint32_t milliamps)
{
    FastLED.setMaxPowerInVoltsAndMilliamps(volts, milliamps);
}

void set_max_power_in_milliwatts(uint32_t powerInmW)
{
    FastLED.setMaxPowerInMilliWatts(powerInmW);
}

void show_at_max_brightness_for_power()
{
    // power management usage is now in FastLED.show, no need for this function
    FastLED.show();
}

void delay_at_max_brightness_for_power(uint16_t ms)
{
    FastLED.delay(ms);
}

FASTLED_NAMESPACE_END
