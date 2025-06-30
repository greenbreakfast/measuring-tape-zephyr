# M5 StickC Plus2

Adds basic support for the M5 StickC Plus2 device.

## Info on Board

- Buy link: https://shop.m5stack.com/products/m5stickc-plus2-esp32-mini-iot-development-kit
- Docs: https://docs.m5stack.com/en/core/M5StickC%20PLUS2
- Uses the **ESP32-PICO-V3-02** SiP

## Implementation Details

- Used the Zephyr v3.7.0 board definition for the previous-gen M5 StickC Plus (based on ESP32-Pico-D4 SiP) as reference 
- Instead of defining the flash partitions for this baord, used Espressif's default partition map for 8MB flash by including the `espressif/partitions_0x1000_default_8M.dtsi` file in the board DTS
- The ESP32 expects a bootloader to be present at flash offset `0x1000`. Compiled Zephyr applications include the MCU Simple Boot boatloader, and must be flashed to flash offset `0x1000` for this board to properly boot