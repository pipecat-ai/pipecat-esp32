#include "main.h"

#include <inttypes.h>
#include <esp_event.h>
#include <esp_log.h>
#include <peer.h>

#ifndef LINUX_BUILD
#include "nvs_flash.h"

// All operations that modify the webrtc state should use this semaphore to ensure thread safety
SemaphoreHandle_t webrtcSemaphore = NULL;

extern "C" void app_main(void) {

  ESP_LOGI(LOG_TAG, "[APP] Startup..");
  ESP_LOGI(LOG_TAG, "[APP] IDF version: %s", esp_get_idf_version());
  heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  webrtcSemaphore = xSemaphoreCreateMutex();

  pipecat_init_screen();
  peer_init();
  pipecat_init_audio_capture();
  pipecat_init_audio_decoder();
  pipecat_init_wifi();
  pipecat_init_webrtc();

  pipecat_screen_system_log("Pipecat ESP32 client initialized\n");

  pipecat_webrtc_run_task();

  while (1) {
    // Add some stats printout to the screen here
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL));
  }
}
#else
int main(void) {
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  peer_init();
  pipecat_webrtc();

  while (1) {
    // Todo: test/fix linux build
    pipecat_webrtc_loop();
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL));
  }
}
#endif
