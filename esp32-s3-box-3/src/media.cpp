#include <bsp/esp-bsp.h>
#include <opus.h>

#include <atomic>
#include <cstring>
#include <esp_timer.h>
#include <freertos/ringbuf.h>

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

// Forward declare for use in set_is_playing
static bool ref_buffer_primed = false;



std::atomic<bool> is_playing = false;
void set_is_playing(int16_t *in_buf, size_t in_samples) {
  bool any_set = false;
  for (size_t i = 0; i < in_samples; i++) {
    if (in_buf[i] != -1 && in_buf[i] != 0 && in_buf[i] != 1) {
      any_set = true;
    }
  }
  bool was_playing = is_playing.load();
  is_playing = any_set;
  
  // Reset buffer priming when playback stops
  if (was_playing && !any_set) {
    ref_buffer_primed = false;
    ESP_LOGD(LOG_TAG, "Playback stopped, reset reference buffer priming");
  }
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

// Circular buffer for speaker reference data (not using FreeRTOS ring buffer)
static int16_t* speaker_circular_buffer = NULL;
static const size_t SPEAKER_BUFFER_SAMPLES = 4096;  // 4096 samples = 8192 bytes
static const size_t MIN_REF_BUFFER_SAMPLES = 2560;  // 2 read cycles worth to prevent underruns
static volatile size_t speaker_write_idx = 0;
static volatile size_t speaker_read_idx = 0;
static volatile size_t speaker_samples_available = 0;
static portMUX_TYPE speaker_buffer_mutex = portMUX_INITIALIZER_UNLOCKED;
// ref_buffer_primed already declared above for use in set_is_playing

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
  // Log packet arrival timing
  static uint64_t last_decode_time = 0;
  uint64_t now = esp_timer_get_time();
  if (last_decode_time > 0) {
    uint64_t delta_ms = (now - last_decode_time) / 1000;
    static int packet_count = 0;
    if (++packet_count % 10 == 0) {
      ESP_LOGI(LOG_TAG, "Audio packet %d: size=%d, delta=%llu ms", packet_count, size, delta_ms);
    }
  }
  last_decode_time = now;
  
  esp_err_t ret;
  int decoded_size =
      opus_decode(opus_decoder, data, size, decoder_buffer, PCM_BUFFER_SIZE, 0);

  if (decoded_size > 0) {
    set_is_playing(decoder_buffer, decoded_size);
    
    // Send to speaker
    if ((ret = esp_codec_dev_write(spk_codec_dev, decoder_buffer,
                                   decoded_size * sizeof(uint16_t))) !=
        ESP_OK) {
      ESP_LOGE(LOG_TAG, "esp_codec_dev_write failed: %s", esp_err_to_name(ret));
    }
    
    // Also send to circular buffer for AEC reference
    if (speaker_circular_buffer != NULL) {
      // Variables for logging outside critical section
      bool log_write = false;
      bool log_overflow = false;
      int write_count_log = 0;
      int overflow_count_log = 0;
      size_t samples_available_log = 0;
      
      portENTER_CRITICAL(&speaker_buffer_mutex);
      
      // Check if we have space
      size_t space_available = SPEAKER_BUFFER_SAMPLES - speaker_samples_available;
      if (space_available >= decoded_size) {
        // Copy samples to circular buffer
        for (int i = 0; i < decoded_size; i++) {
          speaker_circular_buffer[speaker_write_idx] = decoder_buffer[i];
          speaker_write_idx = (speaker_write_idx + 1) % SPEAKER_BUFFER_SAMPLES;
        }
        speaker_samples_available += decoded_size;
        
        // Prepare logging info
        static int write_count = 0;
        if (++write_count % 50 == 0) {
          log_write = true;
          write_count_log = write_count;
          samples_available_log = speaker_samples_available;
        }
      } else {
        // Buffer overflow
        static int overflow_count = 0;
        if (++overflow_count % 100 == 0) {
          log_overflow = true;
          overflow_count_log = overflow_count;
        }
      }
      
      portEXIT_CRITICAL(&speaker_buffer_mutex);
      
      // Do logging outside critical section
      if (log_write) {
        ESP_LOGI(LOG_TAG, "Speaker buffer writes: %d, samples: %d, available: %d", 
                 write_count_log, decoded_size, samples_available_log);
      }
      if (log_overflow) {
        ESP_LOGW(LOG_TAG, "Speaker buffer overflow count: %d", overflow_count_log);
      }
    } else {
      ESP_LOGW(LOG_TAG, "Speaker circular buffer is NULL!");
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
  
  // Create circular buffer for speaker reference data
  speaker_circular_buffer = (int16_t *)heap_caps_malloc(SPEAKER_BUFFER_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (speaker_circular_buffer == NULL) {
    ESP_LOGE(LOG_TAG, "Failed to allocate speaker circular buffer");
    return;
  }
  ESP_LOGI(LOG_TAG, "Speaker circular buffer created, size=%d samples", SPEAKER_BUFFER_SAMPLES);
  
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

  // Don't zero out microphone input - let AEC handle echo cancellation
  // if (is_playing) {
  //   memset(read_buffer, 0, PCM_BUFFER_SIZE);
  // }

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
      
      // Get reference data from speaker circular buffer
      bool have_ref_data = false;
      int16_t ref_chunk[256];  // Temporary buffer for reference data
      
      if (speaker_circular_buffer != NULL) {
        // Variables for logging outside critical section
        bool log_buffer_state = false;
        bool log_primed = false;
        bool log_underrun = false;
        size_t samples_for_log = 0;
        int underrun_type = 0;  // 0=none, 1=remaining chunks, 2=current chunk
        size_t needed_for_log = 0;
        
        portENTER_CRITICAL(&speaker_buffer_mutex);
        
        // Debug log buffer state occasionally
        static int buffer_check_count = 0;
        if (++buffer_check_count % 100 == 0) {
          log_buffer_state = true;
          samples_for_log = speaker_samples_available;
        }
        
        // Only start using reference data if we have enough buffered
        if (!ref_buffer_primed && speaker_samples_available >= MIN_REF_BUFFER_SAMPLES) {
          ref_buffer_primed = true;
          log_primed = true;
          samples_for_log = speaker_samples_available;
        }
        
        // Only try to get reference data if buffer is primed
        if (ref_buffer_primed) {
          // Check if we have enough samples for this chunk
          if (speaker_samples_available >= aec_chunk_size) {
            // Read samples from circular buffer
            for (int i = 0; i < aec_chunk_size; i++) {
              ref_chunk[i] = speaker_circular_buffer[speaker_read_idx];
              speaker_read_idx = (speaker_read_idx + 1) % SPEAKER_BUFFER_SAMPLES;
            }
            speaker_samples_available -= aec_chunk_size;
            have_ref_data = true;
            
            // Check if we're running too low for remaining chunks
            int remaining_chunks = num_chunks - chunk - 1;
            size_t remaining_samples_needed = remaining_chunks * aec_chunk_size;
            if (speaker_samples_available < remaining_samples_needed) {
              ref_buffer_primed = false;
              log_underrun = true;
              underrun_type = 1;
              samples_for_log = speaker_samples_available;
              needed_for_log = remaining_samples_needed;
            }
          } else {
            // Not enough samples, unprime
            ref_buffer_primed = false;
            log_underrun = true;
            underrun_type = 2;
            samples_for_log = speaker_samples_available;
            needed_for_log = aec_chunk_size;
          }
        }
        
        portEXIT_CRITICAL(&speaker_buffer_mutex);
        
        // Do logging outside critical section
        if (log_buffer_state) {
          ESP_LOGI(LOG_TAG, "Circular buffer state: %d samples available, chunk %d/%d", 
                   samples_for_log, chunk, num_chunks);
        }
        if (log_primed) {
          ESP_LOGI(LOG_TAG, "Reference buffer primed with %d samples", samples_for_log);
        }
        if (log_underrun) {
          if (underrun_type == 1) {
            ESP_LOGW(LOG_TAG, "Reference buffer underrun (chunk %d/%d, need %d samples for remaining, have %d)", 
                     chunk, num_chunks, needed_for_log, samples_for_log);
          } else {
            ESP_LOGW(LOG_TAG, "Reference buffer underrun (chunk %d/%d, need %d samples for current, have %d)", 
                     chunk, num_chunks, needed_for_log, samples_for_log);
          }
        }
      } else {
        static bool logged_null = false;
        if (!logged_null) {
          ESP_LOGW(LOG_TAG, "Speaker circular buffer is NULL during read!");
          logged_null = true;
        }
      }
      
      if (have_ref_data) {
        // Use actual speaker reference data
        for (int i = 0; i < aec_chunk_size; i++) {
          aec_in_buffer[i * 2] = mic_ptr[i];      // Microphone channel
          aec_in_buffer[i * 2 + 1] = ref_chunk[i]; // Reference channel (actual speaker data)
        }
        
        // Log successful reads occasionally
        static int ref_success_count = 0;
        if (++ref_success_count % 50 == 0) {
          ESP_LOGI(LOG_TAG, "Using speaker reference data, count: %d", ref_success_count);
        }
      } else {
        // No reference data available, use zeros
        for (int i = 0; i < aec_chunk_size; i++) {
          aec_in_buffer[i * 2] = mic_ptr[i];      // Microphone channel  
          aec_in_buffer[i * 2 + 1] = 0;           // Reference channel (zeros as fallback)
        }
        // Log when we don't have reference data (only occasionally to avoid spam)
        static int no_ref_count = 0;
        if (++no_ref_count % 100 == 0) {
          ESP_LOGW(LOG_TAG, "No speaker reference data available count: %d", no_ref_count);
        }
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
