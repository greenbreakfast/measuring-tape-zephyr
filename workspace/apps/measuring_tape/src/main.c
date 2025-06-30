#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#define DEBOUNCE_DELAY_MS 50
static const int32_t sleep_time_ms = 1000;

static const struct device *const vl53l1x = DEVICE_DT_GET(DT_ALIAS(distance_sensor));
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(trigger_button), gpios);

static struct gpio_callback btn_cb_data;
static struct k_work_delayable button_work;

// forward declarations
int get_distance(int32_t *distance_mm);

// GPIO ISR
void button_isr(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins)
{
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_DELAY_MS));
}

void sensor_work_handler(struct k_work *work) {
    int ret;
    int button_state;
    int32_t distance_mm;

    button_state = gpio_pin_get_dt(&btn);
    // only read distance if button is still pressed (after debounce delay)
    if (button_state) {
        ret = get_distance(&distance_mm);
        if (ret < 0) {
            printk("ERROR: could not get distance, error: %d\n", ret);
        } 
        else {
            printk("Distance: %d mm\n", distance_mm);
        }
    }
    
}

int get_distance(int32_t *distance_mm) {
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

int main (void)
{
    int ret;
    // Initialize work item
    k_work_init_delayable(&button_work, sensor_work_handler);

    // initialize sensor
    if (!device_is_ready(vl53l1x)) {
        printk("ERROR: device is not ready: %s\n", vl53l1x->name);
        return 0;
    }

    // initialize button and isr
    if (!gpio_is_ready_dt(&btn)) {
        printk("ERROR: button not ready\r\n");
        return 0;
    }

    // Set the button as input (apply extra flags if needed)
    ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if (ret < 0) {
        printk("ERROR: could not set button as input\r\n");
        return 0;
    }

    // Configure the interrupt
    ret = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("ERROR: could not configure button as interrupt source\r\n");
        return 0;
    }

    // Connect callback function (ISR) to interrupt source
    gpio_init_callback(&btn_cb_data, button_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb_data);

    while (1) {
        // before ISR:
        // int32_t distance;
        // ret = getDistance(&distance);
        // k_msleep(sleep_time_ms);

        // after ISR:
        k_sleep(K_FOREVER);
    }

    return 0;
}