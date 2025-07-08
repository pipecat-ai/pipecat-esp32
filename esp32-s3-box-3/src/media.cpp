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

// AEC control
static bool aec_enabled = true;  // Can be set to false to bypass AEC

std::atomic<bool> is_playing = false;

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
// This is a sliding window buffer, not a queue - we read recent data for AEC
static int16_t speaker_circular_buffer[960];   // 3 Opus frames (60ms @ 16kHz)
static volatile size_t speaker_write_idx = 0;
static volatile size_t speaker_read_idx = 0;
static volatile size_t speaker_buffer_count = 0;  // Number of samples in buffer
static volatile bool speaker_buffer_active = false; // True when we're receiving audio
static portMUX_TYPE speaker_buffer_mutex = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t last_buffer_read_time = 0;  // Track when buffer was last read

// Function to update playback state and reset buffer when playback stops
void set_is_playing(int16_t *in_buf, size_t in_samples) {
  bool any_set = false;
  for (size_t i = 0; i < in_samples; i++) {
    if (in_buf[i] != -1 && in_buf[i] != 0 && in_buf[i] != 1) {
      any_set = true;
    }
  }
  bool was_playing = is_playing.load();
  is_playing = any_set;
  
  // Reset buffer state when playback stops
  if (was_playing && !any_set) {
    portENTER_CRITICAL(&speaker_buffer_mutex);
    speaker_buffer_active = false;
    speaker_read_idx = speaker_write_idx; // Reset read to match write
    speaker_buffer_count = 0;
    last_buffer_read_time = 0;  // Reset read time
    portEXIT_CRITICAL(&speaker_buffer_mutex);
    ESP_LOGD(LOG_TAG, "Playback stopped, reset speaker buffer");
  }
}

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
    packet_count++;
    
    // Log every packet for first 20, then every 10th
    if (packet_count <= 20 || packet_count % 10 == 0) {
      // Get current buffer state for correlation
      size_t current_write_idx, current_read_idx, current_count;
      portENTER_CRITICAL(&speaker_buffer_mutex);
      current_write_idx = speaker_write_idx;
      current_read_idx = speaker_read_idx;
      current_count = speaker_buffer_count;
      portEXIT_CRITICAL(&speaker_buffer_mutex);
      
      ESP_LOGI(LOG_TAG, "[ARRIVAL] pkt#%d: size=%d, delta=%llums, buffer=%d samples, w=%d r=%d", 
               packet_count, size, delta_ms, current_count, current_write_idx, current_read_idx);
    }
  }
  last_decode_time = now;
  
  // Decode Opus frame to get the size (or use fixed size if predictable)
  int decoded_size = opus_decode(opus_decoder, data, size, decoder_buffer, PCM_BUFFER_SIZE, 0);

  if (decoded_size > 0) {
    // FIRST: Write to circular buffer for AEC reference (before any other processing)
    portENTER_CRITICAL(&speaker_buffer_mutex);
      
      // Write to circular buffer
      size_t space_available = 960 - speaker_buffer_count;
      size_t samples_to_write = decoded_size;
      
      // If we don't have enough space, drop oldest samples
      if (samples_to_write > space_available) {
        size_t samples_to_drop = samples_to_write - space_available;
        speaker_read_idx = (speaker_read_idx + samples_to_drop) % 960;
        speaker_buffer_count -= samples_to_drop;
      }
      
      // Write new samples
      for (int i = 0; i < samples_to_write; i++) {
        speaker_circular_buffer[speaker_write_idx] = decoder_buffer[i];
        speaker_write_idx = (speaker_write_idx + 1) % 960;
      }
      speaker_buffer_count += samples_to_write;
      speaker_buffer_active = true;
      
      // Get samples in buffer after write (while still in critical section)
      size_t samples_in_buffer = speaker_buffer_count;
      
      portEXIT_CRITICAL(&speaker_buffer_mutex);
      
      // Log timing relative to last read from this buffer
      uint64_t time_since_last_read = 0;
      if (last_buffer_read_time > 0 && now > last_buffer_read_time) {
        time_since_last_read = (now - last_buffer_read_time) / 1000;
      } else {
        time_since_last_read = 0;  // First write or invalid timestamp
      }
      
      // More frequent logging for debugging
      static int write_count = 0;
      write_count++;
      if (write_count <= 20 || write_count % 10 == 0) {
        // Log the actual count, not calculated value
        ESP_LOGI(LOG_TAG, "[WRITE] cnt=%d: +%d samples, buffer=%d, since_read=%llums (w=%d r=%d)", 
                 write_count, decoded_size, samples_in_buffer, time_since_last_read,
                 speaker_write_idx, speaker_read_idx);
      }
    
    // THEN: Process for playback
    set_is_playing(decoder_buffer, decoded_size);
    
    // Send to speaker
    esp_err_t ret;
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
static uint64_t total_wait_time = 0;
static uint32_t cycle_count = 0;
static uint32_t aec_skip_count = 0;
static uint64_t last_frame_send_time = 0;

// Frame rate monitoring
static uint64_t first_frame_time = 0;
static uint32_t frames_sent = 0;
static float current_frame_rate = 0.0;
static const uint32_t MIN_FRAMES_FOR_MEASUREMENT = 50;  // Measure over 50 frames (~1 second)
static const float TARGET_FRAME_RATE = 50.0;  // 50 fps (20ms per frame)
static const float MIN_ACCEPTABLE_FRAME_RATE = 48.0;  // Allow 4% tolerance

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

static int16_t *aec_in_buffer = NULL;
static int16_t *aec_out_buffer = NULL;
static afe_aec_handle_t *aec_handle = NULL;

void pipecat_init_aec() {
  // Try a simpler approach - create AEC without the task configuration struct
  ESP_LOGI(LOG_TAG, "Initializing AEC");
  
  // Speaker circular buffer is now statically allocated
  ESP_LOGI(LOG_TAG, "Speaker circular buffer ready, size=960 samples");
  
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
  static uint64_t last_call_end_time = 0;
  uint64_t function_start_time = esp_timer_get_time();
  
  // Log time since last call
  if (last_call_end_time > 0) {
    uint64_t time_between_calls = (function_start_time - last_call_end_time) / 1000;  // ms
    static int between_calls_log_count = 0;
    between_calls_log_count++;
    if (between_calls_log_count <= 20 || between_calls_log_count % 10 == 0) {
      ESP_LOGI(LOG_TAG, "[CALL_GAP] Time between calls: %llums (call #%d)", 
               time_between_calls, between_calls_log_count);
    }
  }
  
  // Wait for (960 samples = 3 Opus frames) in our speaker data reference buffer to handle codec burst pattern
  uint64_t wait_start = esp_timer_get_time();
  int initial_samples = 0;
  int wait_iterations = 0;
  while (true) {
    portENTER_CRITICAL(&speaker_buffer_mutex);
    int samples_available = 0;
    if (speaker_buffer_active) {
      samples_available = speaker_buffer_count;
    }
    portEXIT_CRITICAL(&speaker_buffer_mutex);
    
    wait_iterations++;
    if (initial_samples == 0) {
      initial_samples = samples_available;
    }
    
    if (samples_available >= 960) {
      break;  // Buffer has 3 Opus frames - ready to handle burst consumption
    }
    
    // Brief delay before checking again
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  uint64_t wait_end = esp_timer_get_time();
  total_wait_time += (wait_end - wait_start);
  
  // Log initial buffer state
  static int call_count = 0;
  call_count++;
  // Get consistent snapshot of buffer state
  size_t log_write_idx, log_read_idx, log_count;
  portENTER_CRITICAL(&speaker_buffer_mutex);
  log_write_idx = speaker_write_idx;
  log_read_idx = speaker_read_idx;
  log_count = speaker_buffer_count;
  portEXIT_CRITICAL(&speaker_buffer_mutex);
  
  ESP_LOGI(LOG_TAG, "[SEND_START] call#%d: initial=%d, wait=%lluus, iters=%d, count=%d, w=%d r=%d", 
           call_count, initial_samples, wait_end - wait_start, wait_iterations,
           log_count, log_write_idx, log_read_idx);
  
  // Buffers for interleaved processing
  static int16_t opus_accumulator[640];  // Accumulate AEC output for Opus encoding
  static int opus_accumulator_pos = 0;
  int16_t mic_chunk_buffer[256];
  int aec_chunk_size = 256;  // AEC processes 256 samples at a time
  
  // Track timing for AEC skip decision
  bool skip_last_aec = false;
  int frame_count = 0;
  
  // Track timing - each read blocks for ~16ms (256 samples @ 16kHz)
  uint64_t loop_start_time = esp_timer_get_time();
  
  // Single loop processing 5 chunks of 256 samples each
  for (int chunk = 0; chunk < 5; chunk++) {
    uint64_t chunk_start_time = esp_timer_get_time();
    // 1. Read 256 samples from microphone
    // Log buffer state before read (for chunk 0 and 1)
    if (chunk <= 1) {
      size_t pre_read_write_idx, pre_read_read_idx, pre_read_count;
      portENTER_CRITICAL(&speaker_buffer_mutex);
      pre_read_write_idx = speaker_write_idx;
      pre_read_read_idx = speaker_read_idx;
      pre_read_count = speaker_buffer_count;
      portEXIT_CRITICAL(&speaker_buffer_mutex);
      
      ESP_LOGI(LOG_TAG, "[PRE_READ] chunk=%d: buffer has %d samples (w=%d r=%d)", 
               chunk, pre_read_count, pre_read_write_idx, pre_read_read_idx);
    }
    
    start_time = esp_timer_get_time();
    if (esp_codec_dev_read(mic_codec_dev, (uint8_t*)mic_chunk_buffer, 256 * sizeof(int16_t)) != ESP_OK) {
      ESP_LOGE(LOG_TAG, "esp_codec_dev_read failed");
      return;
    }
    end_time = esp_timer_get_time();
    total_read_time += (end_time - start_time);
    
    // Log if a long read happened (indicating actual blocking)
    uint64_t read_duration = (end_time - start_time) / 1000;
    if (read_duration > 10) {  // More than 10ms suggests actual blocking
      ESP_LOGI(LOG_TAG, "[BLOCKED_READ] chunk=%d: read took %llums", chunk, read_duration);
    }
    
    // 2. Process with AEC or bypass
    if (aec_handle != NULL && aec_enabled && !(chunk == 4 && skip_last_aec)) {
      // Get reference data from speaker circular buffer
      int16_t ref_chunk[256];
      int samples_available = 0;
      bool log_underrun = false;
      static int underrun_count = 0;
      static int ref_success_count = 0;
      
      portENTER_CRITICAL(&speaker_buffer_mutex);
        
        if (speaker_buffer_active) {
          samples_available = speaker_buffer_count;
          
          if (samples_available >= aec_chunk_size) {
            // We have enough data, read it
            for (int i = 0; i < aec_chunk_size; i++) {
              ref_chunk[i] = speaker_circular_buffer[speaker_read_idx];
              speaker_read_idx = (speaker_read_idx + 1) % 960;
            }
            speaker_buffer_count -= aec_chunk_size;
          } else if (samples_available > 0) {
            // Not enough data, read what we have and pad with zeros
            int i;
            for (i = 0; i < samples_available; i++) {
              ref_chunk[i] = speaker_circular_buffer[speaker_read_idx];
              speaker_read_idx = (speaker_read_idx + 1) % 960;
            }
            speaker_buffer_count = 0;  // We consumed all available samples
            for (; i < aec_chunk_size; i++) {
              ref_chunk[i] = 0;
            }
            
            underrun_count++;
            // Log every underrun for first 20, then every 10th
            if (underrun_count <= 20 || underrun_count % 10 == 0) {
              log_underrun = true;
            }
          } else {
            // No data at all - don't advance read pointer, just use zeros
            for (int i = 0; i < aec_chunk_size; i++) {
              ref_chunk[i] = 0;
            }
            
            underrun_count++;
            // Log every underrun for first 20, then every 10th
            if (underrun_count <= 20 || underrun_count % 10 == 0) {
              log_underrun = true;
            }
          }
        } else {
          // Buffer not active, use zeros
          for (int i = 0; i < aec_chunk_size; i++) {
            ref_chunk[i] = 0;
          }
        }
        
        portEXIT_CRITICAL(&speaker_buffer_mutex);
        
        // Update last read time if we read any data
        // Update last read time if we successfully read data
        if (speaker_buffer_active && samples_available >= aec_chunk_size) {
          last_buffer_read_time = esp_timer_get_time();
        } else if (speaker_buffer_active && samples_available > 0) {
          // Even partial reads update the time
          last_buffer_read_time = esp_timer_get_time();
        }
        
        if (log_underrun) {
          uint64_t time_since_loop_start = (esp_timer_get_time() - loop_start_time) / 1000;
          // Get consistent state outside critical section
          size_t log_w_idx, log_r_idx, log_cnt;
          portENTER_CRITICAL(&speaker_buffer_mutex);
          log_w_idx = speaker_write_idx;
          log_r_idx = speaker_read_idx;
          log_cnt = speaker_buffer_count;
          portEXIT_CRITICAL(&speaker_buffer_mutex);
          
          ESP_LOGW(LOG_TAG, "[UNDERRUN] #%d: chunk=%d, only %d/%d samples, %llums since loop start, count=%d, w=%d r=%d", 
                   underrun_count, chunk, samples_available, aec_chunk_size, 
                   time_since_loop_start, log_cnt, log_w_idx, log_r_idx);
        }
      
      // Create interleaved AEC input
      for (int i = 0; i < aec_chunk_size; i++) {
        aec_in_buffer[i * 2] = mic_chunk_buffer[i];     // Mic
        aec_in_buffer[i * 2 + 1] = ref_chunk[i];        // Reference
      }
      
      // Run AEC
      start_time = esp_timer_get_time();
      int ret = afe_aec_process(aec_handle, aec_in_buffer, aec_out_buffer);
      end_time = esp_timer_get_time();
      total_aec_time += (end_time - start_time);
      
      if (ret < 0) {
        ESP_LOGE(LOG_TAG, "afe_aec_process failed: return value %d", ret);
        // Fall back - copy mic input directly
        memcpy(opus_accumulator + opus_accumulator_pos, mic_chunk_buffer, 256 * sizeof(int16_t));
      } else {
        // Copy AEC output to accumulator
        memcpy(opus_accumulator + opus_accumulator_pos, aec_out_buffer, 256 * sizeof(int16_t));
      }
      // ---- 
      // DEBUG: Just copy mic data
      // memcpy(opus_accumulator + opus_accumulator_pos, mic_chunk_buffer, 256 * sizeof(int16_t));
      
      // Log successful reads more frequently for debugging
      ref_success_count++;
      if (ref_success_count <= 10 || ref_success_count % 20 == 0) {
        uint64_t chunk_duration = (esp_timer_get_time() - chunk_start_time) / 1000;
        ESP_LOGI(LOG_TAG, "[READ_OK] chunk=%d, got %d samples, chunk took %llums", 
                 chunk, aec_chunk_size, chunk_duration);
      }
    } else {
      // AEC disabled or skipped - copy mic input directly
      memcpy(opus_accumulator + opus_accumulator_pos, mic_chunk_buffer, 256 * sizeof(int16_t));
      
      if (chunk == 4 && skip_last_aec) {
        static int skip_log_count = 0;
        if (++skip_log_count % 10 == 0) {
          ESP_LOGI(LOG_TAG, "Skipped AEC on 5th chunk to catch up, count: %d", skip_log_count);
        }
      }
    }
    
    opus_accumulator_pos += 256;
    
    // Note: esp_codec_dev_read blocks, providing natural pacing
    
    // 3. Encode and send Opus frames when we have enough samples
    while (opus_accumulator_pos >= 320) {
      start_time = esp_timer_get_time();
      int encoded_size = opus_encode(opus_encoder, opus_accumulator, 320,
                                    encoder_output_buffer, OPUS_BUFFER_SIZE);
      end_time = esp_timer_get_time();
      total_encode_time += (end_time - start_time);
      
      start_time = esp_timer_get_time();
      peer_connection_send_audio(peer_connection, encoder_output_buffer, encoded_size);
      end_time = esp_timer_get_time();
      total_send_time += (end_time - start_time);
      
      frame_count++;
      
      // Frame rate tracking
      uint64_t now = esp_timer_get_time();
      if (first_frame_time == 0) {
        first_frame_time = now;
        frames_sent = 0;
      }
      
      frames_sent++;
      
      // Check frame rate after minimum frames
      if (frames_sent >= MIN_FRAMES_FOR_MEASUREMENT) {
        uint64_t elapsed_us = now - first_frame_time;
        float elapsed_seconds = elapsed_us / 1000000.0f;
        current_frame_rate = frames_sent / elapsed_seconds;
        
        if (current_frame_rate < MIN_ACCEPTABLE_FRAME_RATE) {
          ESP_LOGW(LOG_TAG, "Frame rate too low: %.1f fps (target: %.1f fps). Falling behind!", 
                   current_frame_rate, TARGET_FRAME_RATE);
          
          // Set flag to skip AEC on next cycle
          // (We'll use this instead of the old timing method)
        }
        
        // Reset counters for next measurement period
        first_frame_time = now;
        frames_sent = 0;
      }
      
      // Old timing check - replaced by frame rate monitoring
      // Keeping the structure in case we want to re-enable with new logic
      /*
      if (frame_count == 3) {
        uint64_t third_frame_sent_time = end_time;
        if (last_frame_send_time > 0) {
          uint64_t elapsed = (third_frame_sent_time - last_frame_send_time) / 1000; // ms
          if (elapsed > 62) {
            skip_last_aec = true;
            aec_skip_count++;
          }
        }
      }
      
      if (frame_count == 4) {
        last_frame_send_time = end_time;
      }
      */
      
      // Shift remaining samples
      opus_accumulator_pos -= 320;
      if (opus_accumulator_pos > 0) {
        memmove(opus_accumulator, opus_accumulator + 320, opus_accumulator_pos * sizeof(int16_t));
      }
    }
  }
  
  // Update statistics
  // Log loop timing
  uint64_t loop_duration = (esp_timer_get_time() - loop_start_time) / 1000;
  static int timing_log_count = 0;
  if (++timing_log_count <= 10 || timing_log_count % 20 == 0) {
    ESP_LOGI(LOG_TAG, "[LOOP_TIME] call#%d: full loop took %llums (expect ~80ms for 5x16ms reads)", 
             call_count, loop_duration);
  }
  
  cycle_count++;
  if (cycle_count % 50 == 0) {
    uint64_t avg_wait_time = total_wait_time / 50;
    uint64_t avg_read_time = total_read_time / 50;
    uint64_t avg_aec_time = total_aec_time / 50;
    uint64_t avg_encode_time = total_encode_time / 50;
    uint64_t avg_send_time = total_send_time / 50;
    
    ESP_LOGI(LOG_TAG, "Audio timing stats (avg over 50 cycles): wait=%llu us, read=%llu us, aec=%llu us, encode=%llu us, send=%llu us, aec_skips=%lu, frame_rate=%.1f fps",
             avg_wait_time, avg_read_time, avg_aec_time, avg_encode_time, avg_send_time, aec_skip_count, current_frame_rate);
    
    // Reset counters
    total_wait_time = 0;
    total_read_time = 0;
    total_aec_time = 0;
    total_encode_time = 0;
    total_send_time = 0;
  }
  
  // Record end time for next call's gap measurement
  last_call_end_time = esp_timer_get_time();
}
