# Measuring Tape Application in Zephyr

Repo with all tools required to build and flash a Zephyr application. The application will, upon a button press, read a distance measurement from VL53L1X sensor and display it on a TFT screen. 

> Repo structure based on [Shawn Hymel's Introduction to Zephyr Github repo](https://github.com/ShawnHymel/introduction-to-zephyr)


## Host System Setup for Interacting with Hardware

### Setup

In top-level dir:

```
python -m venv venv
source venv/bin/activate
python -m pip install pyserial==3.5 esptool==4.8.1
```

### Activating venv

```
source venv/bin/activate
```

This needs to be done before interacting with any boards (flashing applications or connecting to serial terminal).


## Flash to Board (from host system)

Steps to flash an application to a device (from the host system) after the application is compiled (in Docker).

Make sure python virtual environemnt is activated:
```
source venv/bin/activate
```

Set a variable to the app name to be flashed:
```
app=measuring_tape_display
```

Find the port assigned to the device and set to a variable:
```
port=/dev/tty.usbserial-14110
```

### Flashing the ESP32S3 DevKitC
```
python -m esptool --port $port --chip auto --baud 921600 --before default_reset --after hard_reset write_flash -u --flash_mode keep --flash_freq 40m --flash_size detect 0x0 workspace/apps/$app/build/zephyr/zephyr.bin
``` 

### Flashing the M5StickC Plus 2

```
python -m esptool --port $port --chip auto --baud 115200 --before default_reset --after hard_reset write_flash -u --flash_mode keep --flash_freq 40m --flash_size detect 0x1000 workspace/apps/$app/build/zephyr/zephyr.bin
```

> Key differences: 
> - Applications on the M5StickC Plus 2 must be flashed to flash addr `0x1000`. Because this device uses the ESP32 (and not the ESP32S3), it expects to read the bootloader (included in the `zephyr.bin` image) at flash offset `0x1000`
> - Lower `115200` baud rate is supported by the ESP32

## Checking Serial Output

```
python -m serial.tools.miniterm "$port" 115200
```
