#define FASTLED_INTERNAL
#include "FastLED.h"
#include "power_mgt.h"

FASTLED_NAMESPACE_BEGIN

//// POWER MANAGEMENT

// the values have been measured with a WS2812B 16x16 LED panel at full brightness

// scale all power values by this factor, 256 is the default
constexpr float kPowerScale = 256.0;
// default voltage
constexpr float kLEDVoltage = 5.0;
// milliamps at 0% and 100% brightness per color
constexpr uint16_t kIdle_mW = kLEDVoltage * 0.812 * kPowerScale;
constexpr uint16_t kRed_mW = (kLEDVoltage * 16.3 * kPowerScale) - kIdle_mW;
constexpr uint16_t kGreen_mW = (kLEDVoltage * 16.4 * kPowerScale) - kIdle_mW;
constexpr uint16_t kBlue_mW = (kLEDVoltage * 16.2 * kPowerScale) - kIdle_mW;

static uint16_t gRed_mW   = kRed_mW;
static uint16_t gGreen_mW = kGreen_mW;
static uint16_t gBlue_mW  = kBlue_mW;
static uint16_t gDark_mW  = kIdle_mW;

// set power consumption in milliwatt per color for 256 LEDs
void set_power_consumption(uint16_t red, uint16_t green, uint16_t blue, uint16_t idle)
{
    gRed_mW = red;
    gGreen_mW = green;
    gBlue_mW = blue;
    gDark_mW = idle;
}

// scale mA to mW for 256 LEDs
void set_power_consumption_mA(float red, float green, float blue, float idle)
{
    set_power_consumption(
        (kLEDVoltage * kPowerScale) * red,
        (kLEDVoltage * kPowerScale) * green,
        (kLEDVoltage * kPowerScale) * blue,
        (kLEDVoltage * kPowerScale) * idle
    );
}

inline static uint32_t _calc_per_color(uint32_t value, uint16_t mW_for256)
{
    return ((value / 255) * mW_for256) >> 8;
    // return (
    //     (value > (255 << 8)) ?
    //         (value / 255) * mW_for256 : // reduce value to 16bit to avoid overflow
    //         (value * mW_for256) / 255
    // ) >> 8;
}

inline static uint32_t _calculate_idle_power_mW(uint16_t numLeds)
{
    return (numLeds * gDark_mW) >> 8;
}

uint32_t calculate_idle_power_mW(uint16_t numLeds)
{
    return _calculate_idle_power_mW(numLeds);
}

uint32_t calculate_idle_power_mW()
{
    uint32_t total_mW = 0;
    CLEDController *pCur = CLEDController::head();
	while(pCur) {
        total_mW += _calculate_idle_power_mW(pCur->size());
		pCur = pCur->next();
	}
    return total_mW;
}

uint32_t calculate_unscaled_power_mW(const CRGB *ledbuffer, uint16_t numLeds)
{
    uint32_t red32 = 0, green32 = 0, blue32 = 0;
    auto p = reinterpret_cast<const uint8_t *>(&ledbuffer[0]);

    // This loop might benefit from an AVR assembly version -MEK
    uint16_t count = numLeds;
    while(count--) {
        red32   += *p++;
        green32 += *p++;
        blue32  += *p++;
    }

    // calculate total power at 100% brightness
    return _calc_per_color(red32, gRed_mW) + _calc_per_color(green32, gGreen_mW) + _calc_per_color(blue32, gBlue_mW);
}

uint8_t calculate_max_brightness_for_power_vmA(const CRGB *ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_V, uint32_t max_power_mA)
{
	return calculate_max_brightness_for_power_mW(ledbuffer, numLeds, target_brightness, max_power_V * max_power_mA);
}

inline static auto _get_brightness(uint8_t target_brightness, uint32_t max_power_mW, uint32_t total_mW, uint32_t idle_mW)
{
    struct result_t {
        uint8_t brightness; // scaled brightness
        uint32_t requested_power_mW; // current power consumption
    };
	uint32_t requested_power_mW = ((total_mW * target_brightness) / 255) + idle_mW;
    if (requested_power_mW > max_power_mW) {
        return result_t{static_cast<uint8_t>((max_power_mW * target_brightness) / requested_power_mW), max_power_mW};
	}
    return result_t{target_brightness, requested_power_mW};
}

uint8_t calculate_max_brightness_for_power_mW(const CRGB *ledbuffer, uint16_t numLeds, uint8_t target_brightness, uint32_t max_power_mW)
{
 	uint32_t total_mW = calculate_unscaled_power_mW(ledbuffer, numLeds);
    return _get_brightness(target_brightness, max_power_mW, total_mW, _calculate_idle_power_mW(numLeds)).brightness;
}

uint8_t _calculate_max_brightness_for_power_mW(uint8_t target_brightness, uint32_t max_power_mW, uint32_t &requested_power_mW)
{
    uint32_t total_mW = 0, idle_mW = 0;
    CLEDController *pCur = CLEDController::head();
	while(pCur) {
        total_mW += calculate_unscaled_power_mW(pCur->leds(), pCur->size());
        idle_mW += _calculate_idle_power_mW(pCur->size());
		pCur = pCur->next();
	}
    auto res = _get_brightness(target_brightness, max_power_mW, total_mW, idle_mW);
    requested_power_mW = res.requested_power_mW;
    return res.brightness;
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
