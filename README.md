# Measuring Tape Application in Zephyr

Repo with all tools required to build and flash a Zephyr application. The application will, upon a button press, read a distance measurement from VL53L1X sensor and display it on a TFT screen. 

> Repo structure and Docker implementation based on [Shawn Hymel's Introduction to Zephyr Github repo](https://github.com/ShawnHymel/introduction-to-zephyr)


# Build Docker Image

Make sure Docker is installed. Then use the Dockerfile to build an image with the Zephyr and ESP32 toolchain.

> These instructions are directly from the [ShawnHymel/introduction-to-zephyr GitHub repo](https://github.com/ShawnHymel/introduction-to-zephyr)

From this directory, build the image (this will take some time):

```sh
docker build -t env-zephyr-espressif -f Dockerfile.espressif .
```

> **NOTE**: If you see an `Unsupported architecture` error, you may need to set the CPU architecture manually. Add `--build-arg TARGETARCH=amd64` (or `arm64`, depending on your CPU) to your `docker build` command.

You can ignore the warning about setting the password as an `ARG` in the Dockerfile. The container is fairly unsecure anyway; I only recommend running it locally when you need it. You will need to change the password and configure *code-server* and *sshd* to be more secure if you want to use it remotely.

# Run Docker container

> These instructions are directly from the [ShawnHymel/introduction-to-zephyr GitHub repo](https://github.com/ShawnHymel/introduction-to-zephyr)

Run the image in *VS Code Server* mode. Note that it mounts the local *workspace/* directory into the container! We also expose ports 3333 (OpenOCD), 2222 (mapped from 22 within the container for SSH), and 8800 (*code-server*).

Linux/macOS:

```
docker run --rm -it -p 3333:3333 -p 2222:22 -p 8800:8800 -v "$(pwd)"/workspace:/workspace -w /workspace env-zephyr-espressif
```

### Connect to Container

> Recommend using SSH from VS Code to connect to the container. Other methods - connecting through a browser or through VS Code Dev Containers - are possible, see the [ShawnHymel/introduction-to-zephyr GitHub repo README](https://github.com/ShawnHymel/introduction-to-zephyr) for more info 

- Open VS Code
- Make sure the Remote - SSH extension is installed
- Connect to `root@localhost:2222`
- Login using the password in the Dockerfile (default: `zephyr`)
- File -> Open Workspace from File -> select the `/zephyr.code-workspace` file
	- This should configure your VS Code workspace with the */workspace* directory mapped from the host along with */opt/toolchains/zephyr* and */opt/toolchains/modules* so you can browse the Zephyr RTOS source files.
- Install the recommended extensions

Use the container to compile Zephyr applications with the `west` tool. See the README files in the `workspace/apps` directories for more information on compilation commands.


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

# Quick Reference Command Cheat-sheet

After above instructions have been done

### Run Docker container

```
docker run --rm -it -p 3333:3333 -p 2222:22 -p 8800:8800 -v "$(pwd)"/workspace:/workspace -w /workspace env-zephyr-espressif
```

### Flash to board (from host system)

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

### Checking serial output

```
python -m serial.tools.miniterm "$port" 115200
```
