# VL53L1X Driver

Code adapted for Zephyr with Claude's help from [Sparkfun's VL53L1X Arduino Library](https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library)

## Why adapt from Arduino?

- Zephyr v3.7.0 has a VL53L1X driver included (`drivers/sensor/st/vl53l1x/`), but it relies on the ST HAL to work. Tried to hack it to work with any board target, but found it to be much work for the outcome
- Decided to port the Arduino library because:
  - It's confirmed to be working
  - Produces a board-agnostic driver
  