# I2C Shell

Application to show I2C shell on device. Useful for debugging device addresses or testing commands.

*All instructions below assume instructions in top-level README were followed previously.*

## Build with west

Run in Docker container

```
west build -p always -b esp32s3_devkitc/esp32s3/procpu -- -DDTC_OVERLAY_FILE=boards/esp32s3_devkitc.overlay 
```

## Flash to board 

See top-level README