// SPDX-License-Identifier: MIT
// Copyright (c) 2026 GMIC AI Inc.

#pragma once

#include "esp_codec_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Bring up I2C, I2S and the ES8311 codec (playback + capture).
void board_init(void);

/// Playback (speaker) codec device handle.
esp_codec_dev_handle_t board_get_playback_handle(void);

/// Capture (microphone) codec device handle.
esp_codec_dev_handle_t board_get_record_handle(void);

#ifdef __cplusplus
}
#endif
