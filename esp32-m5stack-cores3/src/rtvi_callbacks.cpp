#include <esp_log.h>

#include "main.h"

static void on_bot_started_speaking() {
  // pipecat_screen_new_log();
}

static void on_bot_stopped_speaking() {
  // pipecat_screen_log("\n");
}

static void on_bot_tts_text(const char *text) {
  // pipecat_screen_log(text);
  // pipecat_screen_log(" ");
}

rtvi_callbacks_t pipecat_rtvi_callbacks = {
    .on_bot_started_speaking = on_bot_started_speaking,
    .on_bot_stopped_speaking = on_bot_stopped_speaking,
    .on_bot_tts_text = on_bot_tts_text,
};
