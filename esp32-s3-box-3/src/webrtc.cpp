#ifndef LINUX_BUILD
#include <driver/i2s_std.h>
#include <opus.h>
#endif

#include <esp_event.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>

#include "main.h"

extern SemaphoreHandle_t webrtcSemaphore;

static PeerConnection *peer_connection = NULL;

#ifndef LINUX_BUILD

static TaskHandle_t xPcTaskHandle = NULL;
static TaskHandle_t xSendAudioTaskHandle = NULL;

void pipecat_send_audio_task(void *user_data) {
  pipecat_init_audio_encoder();

  while (1) {
    pipecat_send_audio(peer_connection);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
#endif

static void pipecat_ondatachannel_onmessage_task(char *msg, size_t len,
                                                 void *userdata, uint16_t sid) {
#ifdef LOG_DATACHANNEL_MESSAGES
  ESP_LOGI(LOG_TAG, "DataChannel Message: %s", msg);
#endif
  pipecat_rtvi_handle_message(msg);
}

static void pipecat_ondatachannel_onopen_task(void *userdata) {
  if (peer_connection_create_datachannel(peer_connection, DATA_CHANNEL_RELIABLE,
                                         0, 0, (char *)"rtvi-ai",
                                         (char *)"") != -1) {
    ESP_LOGI(LOG_TAG, "DataChannel created");
  } else {
    ESP_LOGE(LOG_TAG, "Failed to create DataChannel");
  }
}

static void pipecat_onconnectionstatechange_task(PeerConnectionState state,
                                                 void *user_data) {
  ESP_LOGI(LOG_TAG, "PeerConnectionState: %s",
           peer_connection_state_to_string(state));

  if (state == PEER_CONNECTION_DISCONNECTED ||
      state == PEER_CONNECTION_CLOSED) {
#ifndef LINUX_BUILD
    esp_restart();
#endif
  } else if (state == PEER_CONNECTION_COMPLETED) {
#ifndef LINUX_BUILD

    ESP_LOGI(LOG_TAG, "Creating send audio task");
    // Todo: move this to a separate init block
    // Why is this stack size so large? Opus encoder?
    xTaskCreatePinnedToCore(pipecat_send_audio_task, "audio_publisher", 24576, NULL, 7, &xSendAudioTaskHandle, 1);
    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);

    pipecat_init_rtvi(peer_connection, &pipecat_rtvi_callbacks);
#endif
  }
}

static void pipecat_on_icecandidate_task(char *description, void *user_data) {
  char *local_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);
  memset(local_buffer, 0, MAX_HTTP_OUTPUT_BUFFER + 1);
  pipecat_http_request(description, local_buffer);
  peer_connection_set_remote_description(peer_connection, local_buffer,
                                         SDP_TYPE_ANSWER);
  free(local_buffer);
}

void pipecat_init_webrtc() {
  PeerConfiguration peer_connection_config = {
      .ice_servers = {},
      .audio_codec = CODEC_OPUS,
      .video_codec = CODEC_NONE,
      .datachannel = DATA_CHANNEL_STRING,
      .onaudiotrack = [](uint8_t *data, size_t size, void *userdata) -> void {
#ifndef LINUX_BUILD
        pipecat_audio_decode(data, size);
#endif
      },
      .onvideotrack = NULL,
      .on_request_keyframe = NULL,
      .user_data = NULL,
  };

  peer_connection = peer_connection_create(&peer_connection_config);
  if (peer_connection == NULL) {
    ESP_LOGE(LOG_TAG, "Failed to create peer connection");
#ifndef LINUX_BUILD
    esp_restart();
#endif
  }

  peer_connection_oniceconnectionstatechange(
      peer_connection, pipecat_onconnectionstatechange_task);
  peer_connection_onicecandidate(peer_connection, pipecat_on_icecandidate_task);
  peer_connection_ondatachannel(peer_connection,
                                pipecat_ondatachannel_onmessage_task,
                                pipecat_ondatachannel_onopen_task, NULL);

  peer_connection_create_offer(peer_connection);
}

void pipecat_webrtc_loop() {
  peer_connection_loop(peer_connection);
}


void peer_connection_task(void* arg) {
  ESP_LOGI(LOG_TAG, "peer_connection_task started");

  while (peer_connection == NULL) {
    ESP_LOGI(LOG_TAG, "peer_connection_task waiting for peer_connection");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }


  for (;;) {
    if (xSemaphoreTake(webrtcSemaphore, portMAX_DELAY)) {
      peer_connection_loop(peer_connection);
      xSemaphoreGive(webrtcSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void pipecat_webrtc_run_task() {
  ESP_LOGI(LOG_TAG, "peer_connection_task starting ...");

  xTaskCreatePinnedToCore(peer_connection_task, "peer_connection", 16384, NULL, 5, &xPcTaskHandle, 0);
}
