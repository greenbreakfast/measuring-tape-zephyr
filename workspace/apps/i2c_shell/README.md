## Host System Setup

### Setup

In introduction-to-zephyr dir:

```
python -m venv venv
source venv/bin/activate
python -m pip install pyserial==3.5 esptool==4.8.1
```

### Activating venv

```
source venv/bin/activate
```


## Build with west

In `01_blink` dir:

```
west build -p always -b esp32s3_devkitc/esp32s3/procpu -- -DDTC_OVERLAY_FILE=boards/esp32s3_devkitc.overlay 
```

## Flash to board (from host system)

Find port and set to variable:
```
port=/dev/tty.usbserial-14110
```

```
python -m esptool --port $port --chip auto --baud 921600 --before default_reset --after hard_reset write_flash -u --flash_mode keep --flash_freq 40m --flash_size detect 0x0 workspace/apps/01_blink/build/zephyr/zephyr.bin
``` 

## Checking serial output

```
python -m serial.tools.miniterm "$port" 115200
```