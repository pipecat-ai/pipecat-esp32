// SPDX-License-Identifier: MIT
// Copyright (c) 2026 GMIC AI Inc.
//
// Board support for a plain ESP32-S3 + ES8311 audio module: one codec doing
// both capture and playback, no screen, no second ADC, no PMIC. There is no
// esp-bsp package for a board like this, so I2C, I2S and the codec are brought
// up directly against the ESP-IDF and esp_codec_dev public APIs.
//
// Use this file as the template for any custom ESP32-S3 audio board: the only
// things that change per board are the pin defines and the codec model.
//
// Reference board: GMIC HA-TOYMD (ESP32-S3, 4 MB flash, 2 MB quad PSRAM,
// single ES8311 codec). Pin map below was read out of the running chip's GPIO
// matrix routing registers over USB-JTAG and then verified on hardware.
//
//   I2C   SDA 17   SCL 18        ES8311 at 7-bit 0x18
//   I2S   MCLK 16  BCLK 9  WS 45  DOUT 8 (to speaker)  DIN 10 (from mic)
//   Amp   enable GPIO 48
//
// 16-bit Philips I2S at 16 kHz, MCLK = 256x sample rate.

#include "board.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "board";

#define BOARD_I2C_SDA  GPIO_NUM_17
#define BOARD_I2C_SCL  GPIO_NUM_18

#define BOARD_I2S_MCLK GPIO_NUM_16
#define BOARD_I2S_BCLK GPIO_NUM_9
#define BOARD_I2S_WS   GPIO_NUM_45
#define BOARD_I2S_DOUT GPIO_NUM_8    // ESP32 -> ES8311 (playback)
#define BOARD_I2S_DIN  GPIO_NUM_10   // ES8311 -> ESP32 (microphone)

#define BOARD_PA_PIN   GPIO_NUM_48   // speaker amplifier enable

#define BOARD_SAMPLE_RATE   16000
#define BOARD_MCLK_MULT     256
#define BOARD_SPEAKER_VOLUME 85
#define BOARD_MIC_GAIN       30.0

static i2c_master_bus_handle_t i2c_bus;
static i2s_chan_handle_t       i2s_tx;
static i2s_chan_handle_t       i2s_rx;
static esp_codec_dev_handle_t  play_dev;
static esp_codec_dev_handle_t  rec_dev;

static esp_err_t init_i2c(void)
{
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = BOARD_I2C_SCL,
        .sda_io_num = BOARD_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &i2c_bus);
}

// The codec will not accept register writes until its clocks are running, so
// I2S is started before the codec is configured and left enabled.
static esp_err_t init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &i2s_tx, &i2s_rx),
                        TAG, "i2s_new_channel failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(BOARD_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BOARD_I2S_MCLK,
            .bclk = BOARD_I2S_BCLK,
            .ws   = BOARD_I2S_WS,
            .dout = BOARD_I2S_DOUT,
            .din  = BOARD_I2S_DIN,
        },
    };
    std_cfg.clk_cfg.mclk_multiple = BOARD_MCLK_MULT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(i2s_tx, &std_cfg),
                        TAG, "i2s init tx failed");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(i2s_rx, &std_cfg),
                        TAG, "i2s init rx failed");
    i2s_channel_enable(i2s_tx);
    i2s_channel_enable(i2s_rx);
    return ESP_OK;
}

// One ES8311 serves both directions: two esp_codec_dev handles share the same
// codec interface and the same I2S data interface.
static esp_err_t init_codec(void)
{
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = I2C_NUM_0,
        .bus_handle = i2c_bus,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(ctrl, ESP_FAIL, TAG, "ES8311 i2c ctrl failed");

    const audio_codec_gpio_if_t *gpio = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio, ESP_FAIL, TAG, "gpio interface failed");

    es8311_codec_cfg_t codec_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .ctrl_if    = ctrl,
        .gpio_if    = gpio,
        .pa_pin     = BOARD_PA_PIN,
        .use_mclk   = true,
        .mclk_div   = BOARD_MCLK_MULT,
        .hw_gain    = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
    };
    const audio_codec_if_t *codec = es8311_codec_new(&codec_cfg);
    ESP_RETURN_ON_FALSE(codec, ESP_FAIL, TAG, "ES8311 init failed");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = I2S_NUM_0,
        .tx_handle = i2s_tx,
        .rx_handle = i2s_rx,
    };
    const audio_codec_data_if_t *data = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data, ESP_FAIL, TAG, "i2s data interface failed");

    esp_codec_dev_cfg_t out_cfg = {
        .codec_if = codec, .data_if = data, .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    play_dev = esp_codec_dev_new(&out_cfg);
    ESP_RETURN_ON_FALSE(play_dev, ESP_FAIL, TAG, "playback device failed");

    esp_codec_dev_cfg_t in_cfg = {
        .codec_if = codec, .data_if = data, .dev_type = ESP_CODEC_DEV_TYPE_IN,
    };
    rec_dev = esp_codec_dev_new(&in_cfg);
    ESP_RETURN_ON_FALSE(rec_dev, ESP_FAIL, TAG, "record device failed");

    esp_codec_dev_set_out_vol(play_dev, BOARD_SPEAKER_VOLUME);
    esp_codec_dev_set_in_gain(rec_dev, BOARD_MIC_GAIN);
    return ESP_OK;
}

void board_init(void)
{
    ESP_LOGI(TAG, "Initializing plain ESP32-S3 + ES8311 board");
    ESP_ERROR_CHECK(init_i2c());
    ESP_ERROR_CHECK(init_i2s());
    ESP_ERROR_CHECK(init_codec());
    ESP_LOGI(TAG, "Board init complete - ES8311 playback + capture ready");
}

esp_codec_dev_handle_t board_get_playback_handle(void) { return play_dev; }
esp_codec_dev_handle_t board_get_record_handle(void)   { return rec_dev; }
