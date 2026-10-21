#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_types.h"
#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCMD3180_I2C_ADDRESS_DEFAULT  0x4CU
#define PCMD3180_I2C_SCL_DEFAULT      GPIO_NUM_27
#define PCMD3180_I2C_SDA_DEFAULT      GPIO_NUM_28

typedef struct {
    i2c_port_num_t i2c_port;
    gpio_num_t scl_gpio;
    gpio_num_t sda_gpio;
    uint32_t i2c_frequency_hz;
    uint8_t i2c_address;
    bool use_internal_areg;
} pcmd3180_config_t;

#define PCMD3180_CONFIG_DEFAULT() {                         \
    .i2c_port = I2C_NUM_0,                                 \
    .scl_gpio = PCMD3180_I2C_SCL_DEFAULT,                  \
    .sda_gpio = PCMD3180_I2C_SDA_DEFAULT,                  \
    .i2c_frequency_hz = 100000U,                           \
    .i2c_address = PCMD3180_I2C_ADDRESS_DEFAULT,           \
    .use_internal_areg = true,                             \
}

/**
 * @brief Initialize PCMD3180 as an 8-slot, 16-bit TDM slave.
 *
 * ESP32-P4 must already be driving the TDM BCLK and FSYNC clocks before this
 * function is called. Passing NULL selects PCMD3180_CONFIG_DEFAULT().
 */
esp_err_t pcmd3180_init(const pcmd3180_config_t *config);

/** Read or write one page-0 register after initialization. */
esp_err_t pcmd3180_write_reg(uint8_t reg, uint8_t value);
esp_err_t pcmd3180_read_reg(uint8_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif
