#include <bsp/esp-bsp.h>
#include <lvgl.h>
#include <string.h>

#include "main.h"

#define SCREEN_TICK_INTERVAL 50
#define MAX_LOG_LINES 10

static lv_obj_t *screen = NULL;
static lv_obj_t *log_container = NULL;
static lv_obj_t *log_label = NULL;
static lv_obj_t *log_labels[MAX_LOG_LINES];
static int log_line_count = 0;

static lv_style_t STYLE_BLUE;
static lv_style_t STYLE_GREEN;
static lv_style_t STYLE_DEFAULT;

static void init_styles() {
  lv_style_init(&STYLE_BLUE);
  lv_style_set_text_color(&STYLE_BLUE, lv_color_hex(0x3333FF));

  lv_style_init(&STYLE_GREEN);
  lv_style_set_text_color(&STYLE_GREEN, lv_color_hex(0x33FF33));

  lv_style_init(&STYLE_DEFAULT);
  lv_style_set_text_color(&STYLE_DEFAULT, lv_color_hex(0x333333));
}

static lv_obj_t *create_scrollable_log(lv_obj_t *parent) {
  lv_obj_t *container = lv_obj_create(parent);
  lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
  lv_obj_align(container, LV_ALIGN_CENTER, 0, 0);

  // Enable vertical scrolling
  lv_obj_set_scroll_dir(container, LV_DIR_VER);
  // Enable flex layout
  lv_obj_set_layout(container, LV_LAYOUT_FLEX);
  // Vertical stacking
  lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
  // Space between rows
  lv_obj_set_style_pad_row(container, 4, 0);
  return container;
}

static lv_obj_t *create_label(lv_obj_t *container, lv_style_t *style) {
  lv_obj_t *label = lv_label_create(container);
  lv_obj_add_style(label, style, 0);
  lv_obj_set_width(label, LV_PCT(100));
  lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(label, "");
  return label;
}

static void create_current_label(lv_obj_t *container, lv_style_t *style) {
  // Delete oldest labels.
  if (log_line_count >= MAX_LOG_LINES) {
    lv_obj_del(log_labels[0]);

    // Shift remaining labels
    for (int i = 1; i < MAX_LOG_LINES; ++i) {
      log_labels[i - 1] = log_labels[i];
    }
    log_line_count--;
  }

  log_label = create_label(container, style);
  log_labels[log_line_count++] = log_label;
}

static void screen_task(void *pvParameter) {
  while (1) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(SCREEN_TICK_INTERVAL));
  }
}

void pipecat_init_screen() {

  bsp_display_start();

  bsp_display_backlight_on();

  bsp_display_lock(0);

  screen = lv_scr_act();

  init_styles();

  log_container = create_scrollable_log(screen);

  xTaskCreatePinnedToCore(screen_task, "Screen Task", 8192, NULL, 1, NULL, 1);

  ESP_LOGI(LOG_TAG, "Display initialized");
}

void pipecat_screen_system_log(const char *text) {
  if (!log_container) {
    return;
  }

  create_current_label(log_container, &STYLE_DEFAULT);
  pipecat_screen_log(text);
}

void pipecat_screen_new_log() {
  if (!log_container) {
    return;
  }

  create_current_label(log_container, &STYLE_BLUE);
}

void pipecat_screen_log(const char *text) {
  if (!log_container) {
    return;
  }

  const char *current_text = lv_label_get_text(log_label);
  size_t current_len = strlen(current_text);
  size_t new_len = current_len + strlen(text) + 1;

  char *combined = (char *)malloc(new_len);
  if (!combined) {
    ESP_LOGW(LOG_TAG, "Failed to allocate memory for log text.");
    return;
  }

  strcpy(combined, current_text);
  strcat(combined, text);

  lv_label_set_text(log_label, combined);
  free(combined);

  lv_obj_scroll_to_view_recursive(log_label, LV_ANIM_OFF);
}
