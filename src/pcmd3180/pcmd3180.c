#include "pcmd3180.h"

#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pcmd3180";

#define PCMD3180_I2C_TIMEOUT_MS       100

#define PCMD3180_REG_PAGE_CFG         0x00U
#define PCMD3180_REG_SW_RESET         0x01U
#define PCMD3180_REG_SLEEP_CFG        0x02U
#define PCMD3180_REG_ASI_CFG0         0x07U
#define PCMD3180_REG_ASI_CFG1         0x08U
#define PCMD3180_REG_ASI_CFG2         0x09U
#define PCMD3180_REG_ASI_CH1          0x0BU
#define PCMD3180_REG_MST_CFG0         0x13U
#define PCMD3180_REG_MST_CFG1         0x14U
#define PCMD3180_REG_ASI_STS          0x15U
#define PCMD3180_REG_CLK_SRC          0x16U
#define PCMD3180_REG_PDMCLK_CFG       0x1FU
#define PCMD3180_REG_PDMIN_CFG        0x20U
#define PCMD3180_REG_GPO_CFG0         0x22U
#define PCMD3180_REG_GPI_CFG0         0x2BU
#define PCMD3180_REG_GPI_CFG1         0x2CU
#define PCMD3180_REG_CH1_CFG0         0x3CU
#define PCMD3180_REG_DSP_CFG0         0x6BU
#define PCMD3180_REG_DSP_CFG1         0x6CU
#define PCMD3180_REG_IN_CH_EN         0x73U
#define PCMD3180_REG_ASI_OUT_CH_EN    0x74U
#define PCMD3180_REG_PWR_CFG          0x75U
#define PCMD3180_REG_DEV_STS0         0x76U
#define PCMD3180_REG_DEV_STS1         0x77U

#define PCMD3180_SLEEP_ENZ            0x01U
#define PCMD3180_SLEEP_AREG_SELECT    0x80U
#define PCMD3180_CH_CFG_STRIDE         5U
#define PCMD3180_CHANNEL_COUNT         8U

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_device;

static esp_err_t raw_write(uint8_t reg, uint8_t value)
{
    ESP_RETURN_ON_FALSE(s_i2c_device != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "I2C device is not initialized");
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(s_i2c_device, data, sizeof(data),
                               PCMD3180_I2C_TIMEOUT_MS);
}

static esp_err_t raw_read(uint8_t reg, uint8_t *value)
{
    ESP_RETURN_ON_FALSE(s_i2c_device != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "I2C device is not initialized");
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "read destination is NULL");
    return i2c_master_transmit_receive(s_i2c_device, &reg, sizeof(reg), value,
                                       sizeof(*value), PCMD3180_I2C_TIMEOUT_MS);
}

static esp_err_t select_page0(void)
{
    return raw_write(PCMD3180_REG_PAGE_CFG, 0x00U);
}

esp_err_t pcmd3180_write_reg(uint8_t reg, uint8_t value)
{
    ESP_RETURN_ON_ERROR(select_page0(), TAG, "select register page 0");
    return raw_write(reg, value);
}

esp_err_t pcmd3180_read_reg(uint8_t reg, uint8_t *value)
{
    ESP_RETURN_ON_ERROR(select_page0(), TAG, "select register page 0");
    return raw_read(reg, value);
}

static esp_err_t wake(bool use_internal_areg)
{
    uint8_t value = PCMD3180_SLEEP_ENZ;
    if (use_internal_areg) {
        value |= PCMD3180_SLEEP_AREG_SELECT;
    }
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_SLEEP_CFG, value), TAG,
                        "wake device");
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

static esp_err_t configure_tdm(void)
{
    /* TDM format, 16-bit words, normal BCLK/FSYNC polarity. */
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_ASI_CFG0, 0x00U), TAG,
                        "configure TDM word format");
    /* TX_OFFSET=0: channel 1 MSB is aligned to the rising edge of FSYNC. */
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_ASI_CFG1, 0x00U), TAG,
                        "configure TDM timing");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_ASI_CFG2, 0x00U), TAG,
                        "configure TDM bus");

    for (uint8_t slot = 0; slot < PCMD3180_CHANNEL_COUNT; ++slot) {
        ESP_RETURN_ON_ERROR(raw_write((uint8_t)(PCMD3180_REG_ASI_CH1 + slot), slot),
                            TAG, "map channel %u to TDM slot %u",
                            (unsigned)(slot + 1U), (unsigned)slot);
    }
    return ESP_OK;
}

static esp_err_t configure_clocks(void)
{
    /* Slave mode, automatic clock configuration and PLL enabled. */
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_MST_CFG0, 0x01U), TAG,
                        "configure clock slave mode");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_MST_CFG1, 0x48U), TAG,
                        "configure clock dividers");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_CLK_SRC, 0x10U), TAG,
                        "configure clock source");
    /* 24-kHz audio uses the 1.4112/1.536-MHz PDM clock range. */
    return raw_write(PCMD3180_REG_PDMCLK_CFG, 0x41U);
}

static esp_err_t configure_channels(void)
{
    /* GPO1..GPO4 output the four PDM clock pairs. */
    for (uint8_t output = 0; output < 4U; ++output) {
        ESP_RETURN_ON_ERROR(raw_write((uint8_t)(PCMD3180_REG_GPO_CFG0 + output),
                                      0x41U),
                            TAG, "configure PDM clock output %u",
                            (unsigned)(output + 1U));
    }

    /* GPI1..GPI4 map to PDM input pairs 1..4, matching the STM32 example. */
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_GPI_CFG0, 0x65U), TAG,
                        "map PDM inputs 1 and 2");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_GPI_CFG1, 0x47U), TAG,
                        "map PDM inputs 3 and 4");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_PDMIN_CFG, 0xf0U), TAG,
                        "configure PDM data edges");

    for (uint8_t channel = 0; channel < PCMD3180_CHANNEL_COUNT; ++channel) {
        const uint8_t base = (uint8_t)(PCMD3180_REG_CH1_CFG0 +
                                       channel * PCMD3180_CH_CFG_STRIDE);
        ESP_RETURN_ON_ERROR(raw_write(base, 0x40U), TAG,
                            "select PDM input for channel %u",
                            (unsigned)(channel + 1U));
        ESP_RETURN_ON_ERROR(raw_write((uint8_t)(base + 2U), 0xC9U), TAG,
                            "set channel %u digital volume",
                            (unsigned)(channel + 1U));
        ESP_RETURN_ON_ERROR(raw_write((uint8_t)(base + 3U), 0x80U), TAG,
                            "set channel %u gain calibration",
                            (unsigned)(channel + 1U));
        ESP_RETURN_ON_ERROR(raw_write((uint8_t)(base + 4U), 0x00U), TAG,
                            "set channel %u phase calibration",
                            (unsigned)(channel + 1U));
    }

    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_DSP_CFG0, 0x01U), TAG,
                        "configure channel DSP");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_DSP_CFG1, 0x00U), TAG,
                        "configure volume DSP");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_IN_CH_EN, 0xFFU), TAG,
                        "enable all input channels");
    return raw_write(PCMD3180_REG_ASI_OUT_CH_EN, 0xFFU);
}

static esp_err_t log_status(void)
{
    uint8_t asi_cfg0;
    uint8_t asi_cfg1;
    uint8_t asi_status;
    uint8_t dev_status0;
    uint8_t dev_status1;

    ESP_RETURN_ON_ERROR(raw_read(PCMD3180_REG_ASI_CFG0, &asi_cfg0), TAG,
                        "read ASI_CFG0");
    ESP_RETURN_ON_ERROR(raw_read(PCMD3180_REG_ASI_CFG1, &asi_cfg1), TAG,
                        "read ASI_CFG1");
    ESP_RETURN_ON_ERROR(raw_read(PCMD3180_REG_ASI_STS, &asi_status), TAG,
                        "read ASI_STS");
    ESP_RETURN_ON_ERROR(raw_read(PCMD3180_REG_DEV_STS0, &dev_status0), TAG,
                        "read DEV_STS0");
    ESP_RETURN_ON_ERROR(raw_read(PCMD3180_REG_DEV_STS1, &dev_status1), TAG,
                        "read DEV_STS1");

    ESP_LOGI(TAG,
             "ready: TDM slave 8 slots x 16-bit, ASI_CFG0=0x%02x "
             "ASI_CFG1=0x%02x ASI_STS=0x%02x DEV_STS0=0x%02x DEV_STS1=0x%02x",
             asi_cfg0, asi_cfg1, asi_status, dev_status0, dev_status1);
    return ESP_OK;
}

static esp_err_t configure_device(bool use_internal_areg)
{
    ESP_RETURN_ON_ERROR(wake(use_internal_areg), TAG, "initial wake");
    ESP_RETURN_ON_ERROR(select_page0(), TAG, "select register page 0");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_SW_RESET, 0x01U), TAG, "software reset");
    vTaskDelay(pdMS_TO_TICKS(3));
    ESP_RETURN_ON_ERROR(wake(use_internal_areg), TAG, "wake after reset");
    ESP_RETURN_ON_ERROR(select_page0(), TAG, "select register page 0");
    ESP_RETURN_ON_ERROR(configure_tdm(), TAG, "configure TDM interface");
    ESP_RETURN_ON_ERROR(configure_clocks(), TAG, "configure clocks");
    ESP_RETURN_ON_ERROR(configure_channels(), TAG, "configure PDM channels");
    ESP_RETURN_ON_ERROR(raw_write(PCMD3180_REG_PWR_CFG, 0x7CU), TAG, "power up PDM and PLL");
    vTaskDelay(pdMS_TO_TICKS(2));
    return log_status();
}

esp_err_t pcmd3180_init(const pcmd3180_config_t *config)
{
    const pcmd3180_config_t default_config = PCMD3180_CONFIG_DEFAULT();
    const pcmd3180_config_t *cfg = config != NULL ? config : &default_config;

    ESP_RETURN_ON_FALSE(s_i2c_bus == NULL && s_i2c_device == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "already initialized");
    ESP_RETURN_ON_FALSE(cfg->i2c_address <= 0x7FU &&
                        GPIO_IS_VALID_OUTPUT_GPIO(cfg->scl_gpio) &&
                        GPIO_IS_VALID_OUTPUT_GPIO(cfg->sda_gpio) &&
                        cfg->scl_gpio != cfg->sda_gpio &&
                        cfg->i2c_frequency_hz > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid I2C configuration");

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = cfg->i2c_port,
        .sda_io_num = cfg->sda_gpio,
        .scl_io_num = cfg->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus), TAG,
                        "create I2C master bus");

    /* A missing slave must never stop USB enumeration or trigger an abort. */
    esp_err_t probe_err = i2c_master_probe(s_i2c_bus, cfg->i2c_address,
                                           PCMD3180_I2C_TIMEOUT_MS);
    if (probe_err != ESP_OK) {
        ESP_LOGE(TAG, "no PCMD3180 at 0x%02x: %s", cfg->i2c_address,
                 esp_err_to_name(probe_err));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return probe_err;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->i2c_address,
        .scl_speed_hz = cfg->i2c_frequency_hz,
        .scl_wait_us = 0,
    };
    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &device_config,
                                               &s_i2c_device);
    if (err != ESP_OK) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    ESP_LOGI(TAG, "I2C port %d, SCL GPIO%d, SDA GPIO%d, address 0x%02x",
             (int)cfg->i2c_port, (int)cfg->scl_gpio, (int)cfg->sda_gpio,
             cfg->i2c_address);

    err = configure_device(cfg->use_internal_areg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configuration failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_i2c_device);
        s_i2c_device = NULL;
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    return err;
}
