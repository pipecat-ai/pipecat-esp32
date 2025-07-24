#include <opus.h>

#include <atomic>
#include <cstring>
#include <cstdio>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>
#include <esp_log.h>

#include "driver/i2c_master.h"
#include <driver/i2s_std.h>
#include "freertos/ringbuf.h"

#include "main.h"

#define SAMPLE_RATE (16000)

#define OPUS_BUFFER_SIZE 1276  // 1276 bytes is recommended by opus_encode
#define PCM_BUFFER_SIZE 640
#define PLAY_BUFFER_SIZE 50

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0

i2c_master_bus_handle_t i2c_bus;
esp_codec_dev_handle_t audio_dev;

void configure_pi4ioe(void) {
    i2c_master_dev_handle_t i2c_device;
    i2c_device_config_t i2c_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x43, // PI4IOE Address
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device));

    auto writeRegister = [=](uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        ESP_ERROR_CHECK(i2c_master_transmit(i2c_device, buffer, 2, 100));
    };

    writeRegister(0x07, 0x00); // Set to high-impedance
    writeRegister(0x0D, 0xFF); // Enable pull-up
    writeRegister(0x03, 0x6E); // Set input=0, output=1
    writeRegister(0x05, 0xFF); // Unmute speaker
    i2c_master_bus_rm_device(i2c_device);
}

void configure_es8311(void) {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };

    i2s_chan_handle_t tx_handle = nullptr, rx_handle = nullptr;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false
        },
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = GPIO_NUM_8,
            .ws = GPIO_NUM_6,
            .dout = GPIO_NUM_5,
            .din = GPIO_NUM_7,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = rx_handle,
        .tx_handle = tx_handle,
    };
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM_1,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
    };
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg),
        .gpio_if = audio_codec_new_gpio(),
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = GPIO_NUM_NC,
        .use_mclk = false,
        .hw_gain = {
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3
        }
    };

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = es8311_codec_new(&es8311_cfg),
        .data_if = audio_codec_new_i2s_data(&i2s_cfg),
    };
    audio_dev = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 1,
        .channel_mask = 0,
        .sample_rate = SAMPLE_RATE,
        .mclk_multiple = 0,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(audio_dev, &fs));
    ESP_ERROR_CHECK(esp_codec_dev_set_in_gain(audio_dev, 30.0));
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(audio_dev, 100));
}

int record_audio(void* dest, int size) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_read(audio_dev, (void*)dest, size));
    return size;
}

void pipecat_init_audio_capture() {
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = GPIO_NUM_38,
        .scl_io_num = GPIO_NUM_39,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));
    configure_pi4ioe();
    configure_es8311();
}

opus_int16 *decoder_buffer = NULL;
RingbufHandle_t decoder_buffer_queue;
std::atomic<int> play_task_buffer_idx = 0;

int play_audio(const void* data, int size) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_write(audio_dev, (void*)data, size));
    return size;
}

static void play_task(void *arg) {
  size_t len;
  uint8_t *play_task_buffer[PLAY_BUFFER_SIZE + 1] = {0};

  while (1) {
      auto audio_buffer = (uint8_t *) xRingbufferReceive(decoder_buffer_queue, &len, portMAX_DELAY);
      play_task_buffer_idx++;

      if (play_task_buffer_idx < PLAY_BUFFER_SIZE) {
          play_task_buffer[play_task_buffer_idx] = audio_buffer;
          continue;
      }

      if (play_task_buffer_idx == PLAY_BUFFER_SIZE) {
          for (auto i = 1; i < PLAY_BUFFER_SIZE; i++) {
              play_audio(play_task_buffer[i], PCM_BUFFER_SIZE);
              vRingbufferReturnItem(decoder_buffer_queue, play_task_buffer[i]);
          }
      }

      play_audio(audio_buffer, PCM_BUFFER_SIZE);
      vRingbufferReturnItem(decoder_buffer_queue, audio_buffer);
  }
}

OpusDecoder *opus_decoder = NULL;
StaticRingbuffer_t rb_struct;

void pipecat_init_audio_decoder() {
  int decoder_error = 0;
  opus_decoder = opus_decoder_create(SAMPLE_RATE, 1, &decoder_error);
  if (decoder_error != OPUS_OK) {
    printf("Failed to create OPUS decoder");
    return;
  }

  decoder_buffer = (opus_int16 *)malloc(PCM_BUFFER_SIZE);

  auto ring_buffer_size = PCM_BUFFER_SIZE * (PLAY_BUFFER_SIZE) + (PLAY_BUFFER_SIZE * 10);
  decoder_buffer_queue = xRingbufferCreateStatic(ring_buffer_size, RINGBUF_TYPE_NOSPLIT, (uint8_t *) malloc(ring_buffer_size), &rb_struct);
  xTaskCreate(play_task, "play_task", 4096, NULL, 5, NULL);
}


std::atomic<bool> is_playing = false;

unsigned int silence_count = 0;

void set_is_playing(int16_t *in_buf, size_t in_samples) {
  bool any_set = false;
  for (size_t i = 0; i < in_samples; i++) {
    if (in_buf[i] != -1 && in_buf[i] != 0 && in_buf[i] != 1) {
      any_set = true;
    }
  }

  if (any_set) {
    silence_count = 0;
  } else {
    silence_count++;
  }

  if (silence_count >= 20 && is_playing) {
    is_playing = false;
    play_task_buffer_idx = 0;
  } else if (any_set && !is_playing) {
    is_playing = true;
  }
}

void pipecat_audio_decode(uint8_t *data, size_t size) {
  int decoded_size = opus_decode(opus_decoder, data, size, decoder_buffer, PCM_BUFFER_SIZE, 0);

  if (decoded_size > 0) {
    set_is_playing(decoder_buffer, decoded_size);
    if (!is_playing) {
      return;
    }

    xRingbufferSend(decoder_buffer_queue, decoder_buffer, PCM_BUFFER_SIZE, 0);
  }
}

OpusEncoder *opus_encoder = NULL;
uint8_t *encoder_output_buffer = NULL;
int16_t *read_buffer = NULL;

void pipecat_init_audio_encoder() {
  int encoder_error;
  opus_encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP,
                                     &encoder_error);
  if (encoder_error != OPUS_OK) {
    printf("Failed to create OPUS encoder");
    return;
  }

  if (opus_encoder_init(opus_encoder, SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP) !=
      OPUS_OK) {
    printf("Failed to initialize OPUS encoder");
    return;
  }

  opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(OPUS_ENCODER_BITRATE));
  opus_encoder_ctl(opus_encoder, OPUS_SET_COMPLEXITY(OPUS_ENCODER_COMPLEXITY));
  opus_encoder_ctl(opus_encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

  read_buffer = (int16_t *)heap_caps_malloc(PCM_BUFFER_SIZE, MALLOC_CAP_DEFAULT);
  encoder_output_buffer = (uint8_t *)malloc(OPUS_BUFFER_SIZE);
}

void pipecat_send_audio(PeerConnection *peer_connection) {
  if (is_playing) {
    memset(read_buffer, 0, PCM_BUFFER_SIZE);
    vTaskDelay(pdMS_TO_TICKS(20));
  } else {
    record_audio(read_buffer, PCM_BUFFER_SIZE);
  }

  auto encoded_size = opus_encode(opus_encoder, (const opus_int16 *)read_buffer,
                                  PCM_BUFFER_SIZE / sizeof(uint16_t),
                                  encoder_output_buffer, OPUS_BUFFER_SIZE);
  peer_connection_send_audio(peer_connection, encoder_output_buffer,
                             encoded_size);
}
