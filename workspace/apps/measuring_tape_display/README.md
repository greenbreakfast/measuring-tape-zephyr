# Measuring Tape 

Application to read distance measurement from VL53L1X sensor and display on TFT screen. A button press will trigger the measurement. 

The display brightness can be controlled with a button press.

Hardware specificis:
- The measurement trigger button is aliased to `trigger-button` in the overlay files in the `boards` directory
- The display brightness can be controlled with a button aliased to `backlight-button` in the overlay files in the `boards` directory

*All instructions below assume instructions in top-level README were followed previously.*

## Build with west

Run in Docker container

### Build for ESP32S3 DevKitC

```
west build -p -b esp32s3_devkitc/esp32s3/procpu -- -DDTC_OVERLAY_FILE=boards/esp32s3_devkitc.overlay
```

### Build for M5StickC Plus2:

```
west build -p always -b m5stickc_plus2/esp32/procpu -- -DBOARD_ROOT=../../ -DDTC_OVERLAY_FILE=boards/m5stickc_plus2.overlay
```

## Flash to board 

See top-level README