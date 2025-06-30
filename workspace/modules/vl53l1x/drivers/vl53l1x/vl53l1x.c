/*
 * VL53L1X Time-of-Flight Distance Sensor Driver for Zephyr
 * Ported from SparkFun Arduino Library
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT custom_vl53l1x

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(VL53L1X, CONFIG_SENSOR_LOG_LEVEL);

/* Register definitions from Arduino library */
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                    0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define SYSTEM__INTERRUPT_CLEAR                             0x0086
#define SYSTEM__MODE_START                                  0x0087
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define GPIO__TIO_HV_STATUS                                 0x0031
#define GPIO_HV_MUX__CTRL                                   0x0030

#define VL53L1X_DEFAULT_ADDRESS                             0x52

/* Default configuration from Arduino library */
static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02,
    0x0a, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
    0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00
};

/* Error type */
typedef int8_t VL53L1X_Error;
#define VL53L1_ERROR_NONE           0
#define VL53L1_ERROR_TIME_OUT      -7

struct vl53l1x_config {
    struct i2c_dt_spec i2c;
#ifdef CONFIG_CUSTOM_VL53L1X_XSHUT
    struct gpio_dt_spec xshut;
#endif
};

struct vl53l1x_data {
    uint8_t i2c_address;
    uint16_t distance_mm;
    bool sensor_ready;
};

/* I2C Communication Functions */
static int vl53l1x_i2c_write(const struct device *dev, uint16_t reg_addr, 
                              const uint8_t *data, uint16_t len)
{
    const struct vl53l1x_config *config = dev->config;
    struct vl53l1x_data *drv_data = dev->data;
    uint8_t buffer[len + 2];
    
    /* Build register address + data buffer */
    buffer[0] = reg_addr >> 8;      /* MSB */
    buffer[1] = reg_addr & 0xFF;    /* LSB */
    memcpy(&buffer[2], data, len);
    
    return i2c_write_dt(&config->i2c, buffer, len + 2);
}

static int vl53l1x_i2c_read(const struct device *dev, uint16_t reg_addr, 
                             uint8_t *data, uint16_t len)
{
    const struct vl53l1x_config *config = dev->config;
    uint8_t reg_buf[2];
    
    reg_buf[0] = reg_addr >> 8;      /* MSB */
    reg_buf[1] = reg_addr & 0xFF;    /* LSB */
    
    return i2c_write_read_dt(&config->i2c, reg_buf, 2, data, len);
}

static int vl53l1x_write_byte(const struct device *dev, uint16_t reg_addr, uint8_t value)
{
    return vl53l1x_i2c_write(dev, reg_addr, &value, 1);
}

static int vl53l1x_read_byte(const struct device *dev, uint16_t reg_addr, uint8_t *value)
{
    return vl53l1x_i2c_read(dev, reg_addr, value, 1);
}

static int vl53l1x_read_word(const struct device *dev, uint16_t reg_addr, uint16_t *value)
{
    uint8_t buffer[2];
    int ret = vl53l1x_i2c_read(dev, reg_addr, buffer, 2);
    if (ret == 0) {
        *value = (buffer[0] << 8) | buffer[1];
    }
    return ret;
}

/* Sensor Functions */
static int vl53l1x_check_boot_state(const struct device *dev)
{
    uint8_t boot_state = 0;
    int timeout = 0;
    
    while (boot_state == 0 && timeout < 100) {
        int ret = vl53l1x_i2c_read(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &boot_state, 1);
        if (ret < 0) {
            return ret;
        }
        if (boot_state == 0) {
            k_msleep(2);
            timeout++;
        }
    }
    
    return (boot_state != 0) ? 0 : -ETIMEDOUT;
}

static int vl53l1x_sensor_init(const struct device *dev)
{
    int ret;
    uint8_t data_ready = 0;
    int timeout = 0;
    
    /* Load default configuration */
    for (int addr = 0x2D; addr <= 0x87; addr++) {
        ret = vl53l1x_write_byte(dev, addr, VL53L1X_DEFAULT_CONFIG[addr - 0x2D]);
        if (ret < 0) {
            LOG_ERR("Failed to write config at addr 0x%02X", addr);
            return ret;
        }
    }
    
    /* Start ranging to initialize */
    ret = vl53l1x_write_byte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
    if (ret < 0) return ret;
    
    ret = vl53l1x_write_byte(dev, SYSTEM__MODE_START, 0x40);
    if (ret < 0) return ret;
    
    /* Wait for first data ready (up to 150ms) */
    while (data_ready == 0 && timeout < 150) {
        uint8_t gpio_status;
        ret = vl53l1x_read_byte(dev, GPIO__TIO_HV_STATUS, &gpio_status);
        if (ret < 0) return ret;
        
        data_ready = gpio_status & 0x01;
        if (data_ready == 0) {
            k_msleep(1);
            timeout++;
        }
    }
    
    if (timeout >= 150) {
        LOG_ERR("Timeout waiting for first measurement");
        return -ETIMEDOUT;
    }
    
    /* Clear interrupt and stop ranging */
    vl53l1x_write_byte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
    vl53l1x_write_byte(dev, SYSTEM__MODE_START, 0x00);
    
    /* Final configuration */
    vl53l1x_write_byte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    vl53l1x_write_byte(dev, 0x0B, 0);
    
    LOG_INF("VL53L1X sensor initialized successfully");
    return 0;
}

static int vl53l1x_get_sensor_id(const struct device *dev, uint16_t *sensor_id)
{
    return vl53l1x_read_word(dev, VL53L1_IDENTIFICATION__MODEL_ID, sensor_id);
}

static int vl53l1x_start_ranging(const struct device *dev)
{
    int ret;
    ret = vl53l1x_write_byte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
    if (ret < 0) return ret;
    
    return vl53l1x_write_byte(dev, SYSTEM__MODE_START, 0x40);
}

static int vl53l1x_stop_ranging(const struct device *dev)
{
    return vl53l1x_write_byte(dev, SYSTEM__MODE_START, 0x00);
}

static int vl53l1x_check_data_ready(const struct device *dev, bool *ready)
{
    uint8_t gpio_status;
    int ret = vl53l1x_read_byte(dev, GPIO__TIO_HV_STATUS, &gpio_status);
    if (ret < 0) return ret;
    
    *ready = (gpio_status & 0x01) != 0;
    return 0;
}

static int vl53l1x_get_distance(const struct device *dev, uint16_t *distance)
{
    return vl53l1x_read_word(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distance);
}

static int vl53l1x_clear_interrupt(const struct device *dev)
{
    return vl53l1x_write_byte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
}

/* Zephyr Sensor API Implementation */
static int vl53l1x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct vl53l1x_data *drv_data = dev->data;
    bool data_ready = false;
    int timeout = 0;
    int ret;
    
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DISTANCE) {
        return -ENOTSUP;
    }
    
    /* Start measurement */
    ret = vl53l1x_start_ranging(dev);
    if (ret < 0) {
        LOG_ERR("Failed to start ranging");
        return ret;
    }
    
    /* Wait for data ready (up to 100ms) */
    while (!data_ready && timeout < 100) {
        ret = vl53l1x_check_data_ready(dev, &data_ready);
        if (ret < 0) {
            vl53l1x_stop_ranging(dev);
            return ret;
        }
        
        if (!data_ready) {
            k_msleep(1);
            timeout++;
        }
    }
    
    if (!data_ready) {
        LOG_ERR("Timeout waiting for measurement");
        vl53l1x_stop_ranging(dev);
        return -ETIMEDOUT;
    }
    
    /* Read distance */
    ret = vl53l1x_get_distance(dev, &drv_data->distance_mm);
    if (ret < 0) {
        LOG_ERR("Failed to read distance");
        vl53l1x_stop_ranging(dev);
        return ret;
    }
    
    /* Clear interrupt and stop ranging */
    vl53l1x_clear_interrupt(dev);
    vl53l1x_stop_ranging(dev);
    
    LOG_DBG("Distance measured: %d mm", drv_data->distance_mm);
    return 0;
}

static int vl53l1x_channel_get(const struct device *dev, enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct vl53l1x_data *drv_data = dev->data;
    
    if (chan != SENSOR_CHAN_DISTANCE) {
        return -ENOTSUP;
    }
    
    /* Convert mm to sensor_value (in meters with micro precision) */
    val->val1 = drv_data->distance_mm / 1000;  /* meters */
    val->val2 = (drv_data->distance_mm % 1000) * 1000;  /* micro-meters */
    
    return 0;
}

static const struct sensor_driver_api vl53l1x_driver_api = {
    .sample_fetch = vl53l1x_sample_fetch,
    .channel_get = vl53l1x_channel_get,
};

/* Device Initialization */
static int vl53l1x_init(const struct device *dev)
{
    const struct vl53l1x_config *config = dev->config;
    struct vl53l1x_data *drv_data = dev->data;
    uint16_t sensor_id;
    int ret;
    
    LOG_INF("Initializing VL53L1X sensor");
    
    /* Check I2C bus ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device not ready");
        return -ENODEV;
    }
    
    /* Initialize GPIO if available */
#ifdef CONFIG_CUSTOM_VL53L1X_XSHUT
    if (config->xshut.port) {
        if (!device_is_ready(config->xshut.port)) {
            LOG_ERR("XSHUT GPIO device not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&config->xshut, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure XSHUT GPIO");
            return ret;
        }
        
        /* Power on sensor */
        gpio_pin_set_dt(&config->xshut, 1);
        k_msleep(10);  /* Boot time */
    }
#endif
    
    /* Wait for sensor boot */
    ret = vl53l1x_check_boot_state(dev);
    if (ret < 0) {
        LOG_ERR("Sensor boot check failed");
        return ret;
    }
    
    /* Verify sensor ID */
    ret = vl53l1x_get_sensor_id(dev, &sensor_id);
    if (ret < 0) {
        LOG_ERR("Failed to read sensor ID");
        return ret;
    }
    
    if (sensor_id != 0xEACC && sensor_id != 0xEBAA) {
        LOG_ERR("Invalid sensor ID: 0x%04X (expected 0xEACC or 0xEBAA)", sensor_id);
        return -ENODEV;
    }
    
    LOG_INF("Found VL53L1X sensor, ID: 0x%04X", sensor_id);
    
    /* Initialize sensor */
    ret = vl53l1x_sensor_init(dev);
    if (ret < 0) {
        LOG_ERR("Sensor initialization failed");
        return ret;
    }
    
    drv_data->sensor_ready = true;
    drv_data->i2c_address = config->i2c.addr;
    
    LOG_INF("VL53L1X driver initialized successfully");
    return 0;
}

/* Device Tree Instantiation */
#define VL53L1X_INIT(inst)                                                     \
    static struct vl53l1x_data vl53l1x_data_##inst;                          \
                                                                              \
    static const struct vl53l1x_config vl53l1x_config_##inst = {             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                   \
        IF_ENABLED(CONFIG_CUSTOM_VL53L1X_XSHUT,                              \
                   (.xshut = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}),)) \
    };                                                                        \
                                                                              \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                       \
                                  vl53l1x_init,                              \
                                  NULL,                                       \
                                  &vl53l1x_data_##inst,                      \
                                  &vl53l1x_config_##inst,                    \
                                  POST_KERNEL,                                \
                                  CONFIG_SENSOR_INIT_PRIORITY,                \
                                  &vl53l1x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(VL53L1X_INIT)