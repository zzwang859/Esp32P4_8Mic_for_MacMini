#include "led_ring_effects.h"

#include <math.h>
#include <stddef.h>

#define LED_RING_BLINK_CYCLE_S 0.1f

static const led_ring_rgb_t DOT_COLOR = {220U, 15U, 231U};
static const led_ring_rgb_t BREATHING_COLOR = {0U, 255U, 0U};
static const led_ring_rgb_t BLINK_COLOR = {207U, 166U, 16U};

typedef struct {
    led_ring_mode_t mode;
    float time_s;
    led_ring_global_params_t global;
    led_ring_dot_params_t dots[LED_RING_MAX_DOTS];
} led_ring_state_t;

led_ring_rgb_t led_ring_rgb[LED_RING_EFFECT_LED_COUNT];

static led_ring_state_t s_state;
static uint8_t s_state_ready;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float wrap_float(float value, float period)
{
    float wrapped = fmodf(value, period);
    return wrapped < 0.0f ? wrapped + period : wrapped;
}

static uint8_t clamp_rgb_float(float value)
{
    if (value <= 0.0f) {
        return 0U;
    }
    if (value >= 255.0f) {
        return 255U;
    }
    return (uint8_t)(value + 0.5f);
}

static float sanitize_cycle(float cycle_s)
{
    return cycle_s > 0.001f ? cycle_s : 0.001f;
}

static led_ring_rgb_t hsv_to_rgb255(float hue, float saturation, float value)
{
    const float h = wrap_float(hue, 1.0f);
    const float s = clamp_float(saturation, 0.0f, 1.0f);
    const float v = clamp_float(value, 0.0f, 1.0f);
    const float scaled = h * 6.0f;
    const int sector = (int)floorf(scaled);
    const float fraction = scaled - (float)sector;
    const float p = v * (1.0f - s);
    const float q = v * (1.0f - fraction * s);
    const float t = v * (1.0f - (1.0f - fraction) * s);
    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;

    switch (sector % 6) {
    case 0:
        red = v;
        green = t;
        blue = p;
        break;
    case 1:
        red = q;
        green = v;
        blue = p;
        break;
    case 2:
        red = p;
        green = v;
        blue = t;
        break;
    case 3:
        red = p;
        green = q;
        blue = v;
        break;
    case 4:
        red = t;
        green = p;
        blue = v;
        break;
    default:
        red = v;
        green = p;
        blue = q;
        break;
    }

    return (led_ring_rgb_t) {
        .r = clamp_rgb_float(red * 255.0f),
        .g = clamp_rgb_float(green * 255.0f),
        .b = clamp_rgb_float(blue * 255.0f),
    };
}

static led_ring_rgb_t rgb_off(void)
{
    return (led_ring_rgb_t) {0U, 0U, 0U};
}

static led_ring_rgb_t scale_rgb(led_ring_rgb_t color, float brightness)
{
    return (led_ring_rgb_t) {
        .r = clamp_rgb_float((float)color.r * brightness),
        .g = clamp_rgb_float((float)color.g * brightness),
        .b = clamp_rgb_float((float)color.b * brightness),
    };
}

static void reset_state(void)
{
    s_state.mode = LED_RING_MODE_RAINBOW;
    s_state.time_s = 0.0f;
    s_state.global.color_hue_deg = 30.0f;
    s_state.global.cycle_s = 6.0f;

    for (uint8_t index = 0U; index < LED_RING_MAX_DOTS; ++index) {
        s_state.dots[index] = (led_ring_dot_params_t) {
            .enabled = index == 0U,
            .color_hue_deg = 30.0f,
            .cycle_s = 3.0f,
            .phase_deg = (float)index * 72.0f,
            .clockwise = 1U,
            .tail_mode = 1U,
        };
    }

}

static void ensure_state_ready(void)
{
    if (!s_state_ready) {
        reset_state();
        s_state_ready = 1U;
    }
}

static led_ring_rgb_t rainbow_chase_pixel(uint16_t index)
{
    float hue = (float)index / (float)LED_RING_EFFECT_LED_COUNT;
    hue += s_state.time_s / sanitize_cycle(s_state.global.cycle_s);
    return hsv_to_rgb255(hue, 1.0f, 0.85f);
}

static led_ring_rgb_t breathing_pixel(void)
{
    const float cycle_s = sanitize_cycle(s_state.global.cycle_s);
    const float t = wrap_float(s_state.time_s, cycle_s);
    const float phase = 2.0f * 3.14159265358979323846f * t / cycle_s;
    const float sine = 0.5f * (sinf(phase - 0.5f * 3.14159265358979323846f) +
                               1.0f);
    const float brightness = 0.04f + 0.96f * sine * sine;
    return scale_rgb(BREATHING_COLOR, brightness);
}

static led_ring_rgb_t blink_pixel(void)
{
    const float phase = wrap_float(s_state.time_s, LED_RING_BLINK_CYCLE_S) /
                        LED_RING_BLINK_CYCLE_S;
    return phase >= 0.5f ? rgb_off() : BLINK_COLOR;
}

static led_ring_rgb_t dot_pixel(uint16_t led_index)
{
    const led_ring_dot_params_t *dot = &s_state.dots[0];
    if (!dot->enabled) {
        return rgb_off();
    }

    float trail_length = (float)LED_RING_EFFECT_LED_COUNT * 0.72f;
    if (trail_length > 13.0f) {
        trail_length = 13.0f;
    }

    const float direction = dot->clockwise ? 1.0f : -1.0f;
    float head = dot->phase_deg / 360.0f +
                 direction * s_state.time_s / sanitize_cycle(dot->cycle_s);
    head = wrap_float(head * (float)LED_RING_EFFECT_LED_COUNT,
                      (float)LED_RING_EFFECT_LED_COUNT);

    float distance;
    if (dot->tail_mode) {
        distance = dot->clockwise
                       ? wrap_float(head - (float)led_index,
                                    (float)LED_RING_EFFECT_LED_COUNT)
                       : wrap_float((float)led_index - head,
                                    (float)LED_RING_EFFECT_LED_COUNT);
    } else {
        distance = fabsf((float)led_index - head);
        if (distance > (float)LED_RING_EFFECT_LED_COUNT - distance) {
            distance = (float)LED_RING_EFFECT_LED_COUNT - distance;
        }
    }

    if (distance <= trail_length) {
        const float brightness =
            powf(1.0f - distance / trail_length, 2.3f);
        return scale_rgb(DOT_COLOR, brightness);
    }
    return rgb_off();
}

static void render_frame(void)
{
    for (uint16_t index = 0U; index < LED_RING_EFFECT_LED_COUNT; ++index) {
        switch (s_state.mode) {
        case LED_RING_MODE_RAINBOW:
            led_ring_rgb[index] = rainbow_chase_pixel(index);
            break;
        case LED_RING_MODE_BREATHING:
            led_ring_rgb[index] = breathing_pixel();
            break;
        case LED_RING_MODE_DOT:
            led_ring_rgb[index] = dot_pixel(index);
            break;
        case LED_RING_MODE_BLINK:
            led_ring_rgb[index] = blink_pixel();
            break;
        case LED_RING_MODE_OFF:
        case LED_RING_MODE_A_GROUP_TEST:
        default:
            led_ring_rgb[index] = rgb_off();
            break;
        }
    }
}

void led_ring_control(const led_ring_control_t *control)
{
    ensure_state_ready();

    if (!control || control->command == LED_RING_CMD_REFRESH) {
        render_frame();
        s_state.time_s += LED_RING_FRAME_DT_S;
        return;
    }

    switch (control->command) {
    case LED_RING_CMD_SET_MODE:
        s_state.mode = control->mode;
        s_state.time_s = 0.0f;
        break;
    case LED_RING_CMD_SET_GLOBAL_PARAMS:
        s_state.global.color_hue_deg = control->global.color_hue_deg;
        s_state.global.cycle_s = sanitize_cycle(control->global.cycle_s);
        break;
    case LED_RING_CMD_SET_DOT_PARAMS:
        if (control->dot_index < LED_RING_MAX_DOTS) {
            s_state.dots[control->dot_index] = control->dot;
            s_state.dots[control->dot_index].cycle_s =
                sanitize_cycle(control->dot.cycle_s);
        }
        break;
    case LED_RING_CMD_RESET_STATE:
        reset_state();
        break;
    case LED_RING_CMD_REFRESH:
    default:
        break;
    }
}
