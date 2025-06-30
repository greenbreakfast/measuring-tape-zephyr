#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "distance_sensor.h"

static const struct device *const vl53l1x = DEVICE_DT_GET(DT_ALIAS(distance_sensor));

int distance_sensor_init(void) {
    // check if sensor is ready
    if (!device_is_ready(vl53l1x)) {
        printk("ERROR: device is not ready: %s\n", vl53l1x->name);
        return -1;
    }
    return 0;
}


// returns 0 on success, error code on failure
// distance_mm is in millimeters
int distance_sensor_get_distance_mm(int32_t *distance_mm) {
    int ret;
    struct sensor_value distance_reading;

    ret = sensor_sample_fetch(vl53l1x);
    if (ret < 0) {
        printk("ERROR: fetching sample for device: %s, error: %d\n", vl53l1x->name, ret);
        return ret;
    }

    ret = sensor_channel_get(
        vl53l1x, 
        SENSOR_CHAN_DISTANCE,
        &distance_reading
    );
    if (ret < 0) {
        printk("ERROR: getting channel for device: %s, error: %d\n", vl53l1x->name, ret);
        return ret;
    }

    *distance_mm = (distance_reading.val1 * 1000) + (distance_reading.val2 / 1000);
    return 0;
}