cmake_minimum_required(VERSION 3.19)

if(NOT IDF_TARGET STREQUAL linux)
  if(NOT DEFINED ENV{WIFI_SSID} OR NOT DEFINED ENV{WIFI_PASSWORD})
    message(FATAL_ERROR "Env variables WIFI_SSID and WIFI_PASSWORD must be set")
  endif()

  add_compile_definitions(WIFI_SSID="$ENV{WIFI_SSID}")
  add_compile_definitions(WIFI_PASSWORD="$ENV{WIFI_PASSWORD}")

  # We are not using video in this app, so set the max NALU size to very small
  if(NOT DEFINED ENV{CONFIG_MAX_NALU_SIZE})
    add_compile_definitions(CONFIG_MAX_NALU_SIZE=32)
  endif()
endif()

if(NOT DEFINED ENV{PIPECAT_SMALLWEBRTC_URL})
  message(FATAL_ERROR "Env variable PIPECAT_SMALLWEBRTC_URL must be set")
endif()

if(DEFINED ENV{LOG_DATACHANNEL_MESSAGES})
  add_compile_definitions(LOG_DATACHANNEL_MESSAGES="1")
endif()

add_compile_definitions(PIPECAT_SMALLWEBRTC_URL="$ENV{PIPECAT_SMALLWEBRTC_URL}")

set(COMPONENTS src)
set(EXTRA_COMPONENT_DIRS "src" "components/srtp" "components/peer" "components/esp-libopus" "esp-adf/components")

if(IDF_TARGET STREQUAL linux)
  add_compile_definitions(LINUX_BUILD=1)
  list(APPEND EXTRA_COMPONENT_DIRS
    $ENV{IDF_PATH}/examples/protocols/linux_stubs/esp_stubs
    "components/esp-protocols/common_components/linux_compat/esp_timer"
    "components/esp-protocols/common_components/linux_compat/freertos"
    )
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(src)
