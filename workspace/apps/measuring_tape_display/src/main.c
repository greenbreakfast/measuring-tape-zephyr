#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "distance_sensor.h"
#include "tft_display.h"

#define DEBOUNCE_DELAY_MS 50
static const int32_t sleep_time_ms = 50;        // Target 20 FPS

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(trigger_button), gpios);
static const struct gpio_dt_spec backlight_btn = GPIO_DT_SPEC_GET(DT_ALIAS(backlight_button), gpios);


K_MUTEX_DEFINE(distance_mutex);

static struct gpio_callback btn_cb_data;
static struct k_work_delayable button_work;

static struct gpio_callback backlight_btn_cb_data;
static struct k_work_delayable backlight_button_work;

static int32_t global_distance_mm = -1;
static int32_t global_backlight_duty_cycle = -1;

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
        ret = distance_sensor_get_distance_mm(&distance_mm);
        if (ret < 0) {
            printk("ERROR: could not get distance, error: %d\n", ret);
        } 
        else {
            // update the global variable
            k_mutex_lock(&distance_mutex, K_FOREVER);
            global_distance_mm = distance_mm;
            k_mutex_unlock(&distance_mutex);
            
            printk("Distance: %d mm\n", distance_mm);
        }
    }
    
}

// Backlight GPIO ISR
void backlight_button_isr(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins)
{
    k_work_reschedule(&backlight_button_work, K_MSEC(DEBOUNCE_DELAY_MS));
}

void backlight_work_handler(struct k_work *work) {
    int ret;
    int button_state;

    button_state = gpio_pin_get_dt(&backlight_btn);
    // only perform work if button is still pressed (after debounce delay)
    if (button_state) {
        global_backlight_duty_cycle -= TFT_DISPLAY_BACKLIGHT_CHANGE_VAL;
        // reset to max if min is exceeded
        if (global_backlight_duty_cycle < TFT_DISPLAY_BACKLIGHT_MIN_VAL) {
            global_backlight_duty_cycle = TFT_DISPLAY_BACKLIGHT_MAX_VAL;
        }
        ret = tft_display_set_backlight_pwm(global_backlight_duty_cycle);
        if (ret != 0) {
            printk("ERROR setting PWM duty cycle: %d\r\n", ret);
            return;
        }
    }
}

// returns -1 if mutex is locked, otherwise returns current distance
int32_t get_latest_distance_nonblocking(void) {
    int ret;
    int32_t distance;
    
    ret = k_mutex_lock(&distance_mutex, K_NO_WAIT);
    if (ret != 0) {
        // mutex locked, try again later
        return -1; 
    }
    distance = global_distance_mm;
    k_mutex_unlock(&distance_mutex);
    return distance;
}

int main (void)
{
    int ret;
    int32_t distance; 
    char buf[11] = {0};

    // initialize the distance sensor
    if (distance_sensor_init() != 0) {
        printk("ERROR: could not initialize distance sensor\n");
        return 0;
    }

    // Initialize work items
    k_work_init_delayable(&button_work, sensor_work_handler);
    k_work_init_delayable(&backlight_button_work, backlight_work_handler);

    // initialize trigger button and isr
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

    // initialize backlight button and isr
    if (!gpio_is_ready_dt(&backlight_btn)) {
        printk("ERROR: button not ready\r\n");
        return 0;
    }

    // Set the button as input (apply extra flags if needed)
    ret = gpio_pin_configure_dt(&backlight_btn, GPIO_INPUT);
    if (ret < 0) {
        printk("ERROR: could not set button as input\r\n");
        return 0;
    }

    // Configure the interrupt
    ret = gpio_pin_interrupt_configure_dt(&backlight_btn, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("ERROR: could not configure button as interrupt source\r\n");
        return 0;
    }

    // Connect callback function (ISR) to interrupt source
    gpio_init_callback(&backlight_btn_cb_data, backlight_button_isr, BIT(backlight_btn.pin));
    gpio_add_callback(backlight_btn.port, &backlight_btn_cb_data);

    // initialize the TFT display
    if (tft_display_init() != 0) {
        printk("ERROR: coudl not initialize TFT display\n");
        return 0;
    }
    // set backlight pwm pulse width
    global_backlight_duty_cycle = TFT_DISPLAY_BACKLIGHT_MIN_VAL;
    ret = tft_display_set_backlight_pwm(global_backlight_duty_cycle);
    if (ret != 0) {
        printk("ERROR setting PWM duty cycle: %d\r\n", ret);
		return 0;
    }

    while (1) {
        // update the measurement if required
        distance = get_latest_distance_nonblocking();
        if (distance >= 0) {
            sprintf(buf, "%d", distance);
            if (tft_display_set_measurement_text(buf) != 0) {
                printk("ERROR: could not set measurement text on display\n");
            }
        }

        // Must be called periodically
		tft_display_task_handler();

        k_msleep(sleep_time_ms);
    }

    return 0;
}