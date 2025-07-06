#include <bsp/esp-bsp.h>
#include <opus.h>

#include <atomic>
#include <cstring>
#include <esp_timer.h>

#include "esp_log.h"
#include "main.h"

#include "esp_afe_aec.h"


#define SAMPLE_RATE (16000)

#define OPUS_BUFFER_SIZE 1276  // 1276 bytes is recommended by opus_encode
#define AEC_CHUNK_SIZE 256  // AEC processes 256 samples at a time for 16kHz
#define OPUS_FRAME_SIZE 320  // Opus frame size in samples
#define COMMON_BUFFER_SAMPLES 1280  // LCM(256, 320) = 1280 samples
#define AEC_CHUNKS_TO_BATCH (COMMON_BUFFER_SAMPLES / AEC_CHUNK_SIZE)  // 5 chunks
#define OPUS_FRAMES_TO_BATCH (COMMON_BUFFER_SAMPLES / OPUS_FRAME_SIZE)  // 4 frames
#define PCM_BUFFER_SIZE (COMMON_BUFFER_SAMPLES * sizeof(int16_t))  // 2560 bytes

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0



std::atomic<bool> is_playing = false;
void set_is_playing(int16_t *in_buf, size_t in_samples) {
  bool any_set = false;
  for (size_t i = 0; i < in_samples; i++) {
    if (in_buf[i] != -1 && in_buf[i] != 0 && in_buf[i] != 1) {
      any_set = true;
    }
  }
  is_playing = any_set;
}

esp_codec_dev_handle_t mic_codec_dev = NULL;
esp_codec_dev_handle_t spk_codec_dev = NULL;

void pipecat_init_audio_capture() {
  mic_codec_dev = bsp_audio_codec_microphone_init();
  spk_codec_dev = bsp_audio_codec_speaker_init();

  esp_codec_dev_set_in_gain(mic_codec_dev, 42.0);
  esp_codec_dev_set_out_vol(spk_codec_dev, 255);

  esp_codec_dev_sample_info_t fs = {
      .bits_per_sample = 16,
      .channel = 1,
      .sample_rate = SAMPLE_RATE,
  };
  esp_codec_dev_open(mic_codec_dev, &fs);
  esp_codec_dev_open(spk_codec_dev, &fs);
}

opus_int16 *decoder_buffer = NULL;
OpusDecoder *opus_decoder = NULL;

void pipecat_init_audio_decoder() {
  int decoder_error = 0;
  opus_decoder = opus_decoder_create(SAMPLE_RATE, 1, &decoder_error);
  if (decoder_error != OPUS_OK) {
    printf("Failed to create OPUS decoder");
    return;
  }

  decoder_buffer = (opus_int16 *)malloc(PCM_BUFFER_SIZE);
}

void pipecat_audio_decode(uint8_t *data, size_t size) {
  esp_err_t ret;
  int decoded_size =
      opus_decode(opus_decoder, data, size, decoder_buffer, PCM_BUFFER_SIZE, 0);

  if (decoded_size > 0) {
    set_is_playing(decoder_buffer, decoded_size);
    if ((ret = esp_codec_dev_write(spk_codec_dev, decoder_buffer,
                                   decoded_size * sizeof(uint16_t))) !=
        ESP_OK) {
      ESP_LOGE(LOG_TAG, "esp_codec_dev_write failed: %s", esp_err_to_name(ret));
    }
  }
}

OpusEncoder *opus_encoder = NULL;
uint8_t *encoder_output_buffer = NULL;
uint8_t *read_buffer = NULL;

// Timing statistics
static uint64_t total_read_time = 0;
static uint64_t total_aec_time = 0;
static uint64_t total_encode_time = 0;
static uint64_t total_send_time = 0;
static uint32_t cycle_count = 0;

// AEC processed buffer
static int16_t *processed_buffer = NULL;

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

  read_buffer =
      (uint8_t *)heap_caps_malloc(PCM_BUFFER_SIZE, MALLOC_CAP_DEFAULT);
  encoder_output_buffer = (uint8_t *)malloc(OPUS_BUFFER_SIZE);
  processed_buffer = (int16_t *)heap_caps_malloc(PCM_BUFFER_SIZE, MALLOC_CAP_DEFAULT);
}


typedef struct {
  int        task_stack;     /*!< Task stack size */
  int        task_prio;      /*!< Task peroid */
  int        task_core;      /*!< The core that task to be created */
  bool       debug_aec;      /*!< debug AEC input data */
  bool       stack_in_ext;   /*!< Try to allocate stack in external memory */
  afe_type_t type;           /*!< The type of afe， AFE_TYPE_SR , AFE_TYPE_VC , AFE_TYPE_VC_8K */
  afe_mode_t mode;           /*!< The mode of afe， AFE_MODE_LOW_COST or AFE_MODE_HIGH_PERF */
  int        filter_length;  /*!< The filter length of aec */
  char      *input_format;   /*!< The input format is as follows: For example, 'MR', 'RMNM'.
                                  'M' means data from microphone, 'R' means data from reference data, 'N' means no data.. */
} aec_stream_cfg_t;

static int16_t *aec_in_buffer = NULL;
static int16_t *aec_out_buffer = NULL;
static afe_aec_handle_t *aec_handle = NULL;

void pipecat_init_aec() {
  // Try a simpler approach - create AEC without the task configuration struct
  ESP_LOGI(LOG_TAG, "Initializing AEC with simple configuration");
  
  // Create AEC with basic parameters (filter_length=4 recommended for ESP32S3)
  aec_handle = afe_aec_create("MR", 4, AFE_TYPE_VC, AFE_MODE_LOW_COST);
  if (aec_handle == NULL) {
    ESP_LOGE(LOG_TAG, "Failed to create AEC");
    return;
  }
  
  ESP_LOGI(LOG_TAG, "AEC created successfully");
  
  int frame_size = afe_aec_get_chunksize(aec_handle);
  int input_size = frame_size * aec_handle->pcm_config.total_ch_num * sizeof(int16_t);
  int output_size = frame_size * sizeof(int16_t);

  // Allocate buffers in PSRAM instead of internal memory
  aec_in_buffer = (int16_t *)heap_caps_aligned_calloc(16, 1, input_size, MALLOC_CAP_SPIRAM);
  aec_out_buffer = (int16_t *)heap_caps_aligned_calloc(16, 1, output_size, MALLOC_CAP_SPIRAM);
  
  if (aec_in_buffer == NULL || aec_out_buffer == NULL) {
    ESP_LOGE(LOG_TAG, "Failed to allocate AEC buffers");
    // Try internal memory as fallback
    if (aec_in_buffer) heap_caps_free(aec_in_buffer);
    if (aec_out_buffer) heap_caps_free(aec_out_buffer);
    
    aec_in_buffer = (int16_t *)heap_caps_aligned_calloc(16, 1, input_size, MALLOC_CAP_INTERNAL);
    aec_out_buffer = (int16_t *)heap_caps_aligned_calloc(16, 1, output_size, MALLOC_CAP_INTERNAL);
    
    if (aec_in_buffer == NULL || aec_out_buffer == NULL) {
      ESP_LOGE(LOG_TAG, "Failed to allocate AEC buffers even in internal memory");
      return;
    }
    ESP_LOGW(LOG_TAG, "AEC buffers allocated in internal memory");
  }
  
  ESP_LOGI(LOG_TAG, "AEC frame size: %d, input size: %d, output size: %d", frame_size, input_size, output_size);
  ESP_LOGI(LOG_TAG, "AEC buffers allocated: in=%p, out=%p", aec_in_buffer, aec_out_buffer);
}


void pipecat_send_audio(PeerConnection *peer_connection) {
  uint64_t start_time, end_time;
  
  // Check if AEC is initialized
  if (aec_handle == NULL) {
    // AEC disabled, bypass processing
    // ESP_LOGW(LOG_TAG, "AEC not initialized, bypassing");
  }
  
  // Measure esp_codec_dev_read time
  start_time = esp_timer_get_time();
  if (esp_codec_dev_read(mic_codec_dev, read_buffer, PCM_BUFFER_SIZE) !=
      ESP_OK) {
    printf("esp_codec_dev_read failed");
    return;
  }
  end_time = esp_timer_get_time();
  total_read_time += (end_time - start_time);

  if (is_playing) {
    memset(read_buffer, 0, PCM_BUFFER_SIZE);
  }

  // AEC processing (skip if disabled)
  if (aec_handle != NULL) {
    // Get AEC chunk size (should be 256 samples for 16kHz)
    int aec_chunk_size = afe_aec_get_chunksize(aec_handle);
    int total_samples = PCM_BUFFER_SIZE / sizeof(uint16_t);  // 640 samples
    int num_chunks = total_samples / aec_chunk_size;  // Should be 2.5, so we'll process 2 chunks
    
    // Debug logging
    static bool first_run = true;
    if (first_run) {
      ESP_LOGI(LOG_TAG, "AEC debug: chunk_size=%d, total_samples=%d, num_chunks=%d", 
               aec_chunk_size, total_samples, num_chunks);
      ESP_LOGI(LOG_TAG, "AEC debug: total_ch_num=%d, sample_rate=%d", 
               aec_handle->pcm_config.total_ch_num, aec_handle->pcm_config.sample_rate);
      ESP_LOGI(LOG_TAG, "AEC debug: input format=%s, mic_num=%d, ref_num=%d",
               "MR", aec_handle->pcm_config.mic_num, aec_handle->pcm_config.ref_num);
      first_run = false;
    }
    
    // Process all chunks
    if (num_chunks != AEC_CHUNKS_TO_BATCH) {
      ESP_LOGW(LOG_TAG, "Unexpected number of AEC chunks: %d (expected %d)", num_chunks, AEC_CHUNKS_TO_BATCH);
    }
    
    // Process AEC in chunks
    for (int chunk = 0; chunk < num_chunks; chunk++) {
      // Prepare input: interleave mic data with zeros (reference channel)
      int16_t *mic_ptr = (int16_t *)read_buffer + (chunk * aec_chunk_size);
      
      // Format: MRMRMR... (interleaved - mic and ref alternating)
      for (int i = 0; i < aec_chunk_size; i++) {
        aec_in_buffer[i * 2] = mic_ptr[i];      // Microphone channel
        aec_in_buffer[i * 2 + 1] = 0;           // Reference channel (zeros for now)
      }
      
      // Run AEC process for this chunk
      start_time = esp_timer_get_time();
      int ret = afe_aec_process(aec_handle, aec_in_buffer, aec_out_buffer);
      end_time = esp_timer_get_time();
      total_aec_time += (end_time - start_time);
      
      if (ret < 0) {
        ESP_LOGE(LOG_TAG, "afe_aec_process failed: return value %d (0x%x) at chunk %d", 
                 ret, ret, chunk);
        // Fall back to bypass on error
        memcpy(processed_buffer, read_buffer, PCM_BUFFER_SIZE);
        return;
      }
      // Log success on first chunk of first run
      if (first_run && chunk == 0) {
        ESP_LOGI(LOG_TAG, "AEC process successful, returned %d bytes", ret);
      }
      
      // Copy AEC output to processed buffer
      memcpy(processed_buffer + (chunk * aec_chunk_size), aec_out_buffer, aec_chunk_size * sizeof(int16_t));
    }
    
    // All samples have been processed through AEC
  } else {
    // AEC disabled, just copy input to processed buffer
    memcpy(processed_buffer, read_buffer, PCM_BUFFER_SIZE);
  }

  // Process Opus frames - we have exactly COMMON_BUFFER_SAMPLES (1280) samples
  // This encodes to exactly OPUS_FRAMES_TO_BATCH (4) frames
  for (int frame = 0; frame < OPUS_FRAMES_TO_BATCH; frame++) {
    // Measure opus_encode time
    start_time = esp_timer_get_time();
    auto encoded_size = opus_encode(opus_encoder, 
                                    processed_buffer + (frame * OPUS_FRAME_SIZE),
                                    OPUS_FRAME_SIZE,
                                    encoder_output_buffer, OPUS_BUFFER_SIZE);
    end_time = esp_timer_get_time();
    total_encode_time += (end_time - start_time);
    
    // Measure peer_connection_send_audio time
    start_time = esp_timer_get_time();
    peer_connection_send_audio(peer_connection, encoder_output_buffer,
                               encoded_size);
    end_time = esp_timer_get_time();
    total_send_time += (end_time - start_time);
  }
  
  // Update cycle count and print statistics every 50 cycles
  cycle_count++;
  if (cycle_count % 50 == 0) {
    uint64_t avg_read_time = total_read_time / 50;
    uint64_t avg_aec_time = total_aec_time / 50;
    uint64_t avg_encode_time = total_encode_time / 50;
    uint64_t avg_send_time = total_send_time / 50;
    
    ESP_LOGI(LOG_TAG, "Audio timing stats (avg over 50 cycles): read=%llu us, aec=%llu us, encode=%llu us, send=%llu us",
             avg_read_time, avg_aec_time, avg_encode_time, avg_send_time);
    
    // Reset counters for next 50 cycles
    total_read_time = 0;
    total_aec_time = 0;
    total_encode_time = 0;
    total_send_time = 0;
  }
}
