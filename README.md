# Pipecat ESP32 Client SDK

## 💻 Platform/Device Support

This SDK has been developed and tested on a `esp32s3` and `linux`. You don't
need any physical hardware to run this SDK, but it will be very limited
(e.g. you won't hear an audio on Linux).

To use it on hardware purchase any of these microcontrollers. Others may
work, but this is what has been developed against.

* [Espressif - ESP32-S3-BOX-3](https://www.digikey.com/short/fb2vjrpn)
* [M5Stack - CoreS3 ESP32S3 loT Development Kit](https://shop.m5stack.com/products/m5stack-cores3-esp32s3-lotdevelopment-kit)

### Custom boards without an `esp-bsp` package

The directories above target dev kits, which come with an `esp-bsp` or
`M5Unified` package that knows the board's audio wiring. Most boards you would
actually ship a product on do not have one.

`esp32-gmic-ha-toymd` is a worked example of a plain ESP32-S3 + ES8311 module -
one codec doing both capture and playback, no screen, no PMIC, no BSP. The only
board-specific file is `src/board.c`, which brings up I2C, I2S and the codec
directly against the ESP-IDF and `esp_codec_dev` APIs. To support a different
custom board, copy that directory and change the pin defines and the codec
model; nothing else in the client changes.

## 📋 Pre-requisites

Clone this repository:

```
git clone --recursive https://github.com/pipecat-ai/pipecat-esp32.git
```

Install the ESP-IDF toolchain following these
[instructions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html).

After that, just open a terminal and load ESP-IDF tools:

```
source PATH_TO_ESP_IDF/export.sh
```

We also need to set a few environment variables before we can build:

```
export WIFI_SSID=foo
export WIFI_PASSWORD=bar
export PIPECAT_SMALLWEBRTC_URL=URL (e.g. http://192.168.1.10:7860/api/offer)
```

where `WIFI_SSID` and `WIFI_PASSWORD` are just needed to connect the device to
the network. `PIPECAT_SMALLWEBRTC_URL` is the URL endpoint to connect to your
Pipecat bot.

## 🛠️ Build

Go inside the directory for your board (e.g. `esp32-s3-box-3`).

The first thing to do is to set the desired target, for example:

```
idf.py --preview set-target esp32s3
```

You can also set `linux` instead of `esp32s3`.

Then, just build:

```
idf.py build
```

If you built for `linux` you can run the binary directly:

```
./build/src.elf
```

## 🔌 Flash the device

If you built for `esp32s3` you can flash your device using the following commands:

### Linux

```
idf.py -p /dev/ttyACM0 flash
```

where `/dev/ttyACM0` is the device where your ESP32 is connected. You can run
`sudo dmesg` to know the device on your system.

On Debian systems, you will want to add your user to the `dialout` group so you
don't need root access.

### macOS

```
idf.py flash
```

## ▶️ Usage

Currently, you can try `pipecat-esp32` with one of the Pipecat foundational
examples. For example, from the Pipecat repository you can run:

```
python examples/foundational/07-interruptible.py --host IP --esp32
```

where `IP` is just your machine IP address (e.g. 192.168.1.10). Then, you would
set the environment variable `PIPECAT_SMALLWEBRTC_URL` as explained above.
