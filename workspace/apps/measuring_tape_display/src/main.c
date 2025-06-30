#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <string.h>

#define DEBOUNCE_DELAY_MS 50
static const int32_t sleep_time_ms = 50;        // Target 20 FPS

static const struct device *const vl53l1x = DEVICE_DT_GET(DT_ALIAS(distance_sensor));
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(trigger_button), gpios);
static const struct gpio_dt_spec backlight_btn = GPIO_DT_SPEC_GET(DT_ALIAS(backlight_button), gpios);
static const struct pwm_dt_spec backlight_pwm = PWM_DT_SPEC_GET(DT_ALIAS(backlight_ctrl));

K_MUTEX_DEFINE(distance_mutex);

static struct gpio_callback btn_cb_data;
static struct k_work_delayable button_work;

static struct gpio_callback backlight_btn_cb_data;
static struct k_work_delayable backlight_button_work;

static int32_t global_distance_mm = -1;
static int32_t global_backlight_duty_cycle = -1;

#ifdef CONFIG_BOARD_M5STICKC_PLUS2
#define BACKLIGHT_MAX_VAL 100
#define BACKLIGHT_MIN_VAL 35
#define BACKLIGHT_CHANGE_VAL 15
#else
#define BACKLIGHT_MAX_VAL 45
#define BACKLIGHT_MIN_VAL 5
#define BACKLIGHT_CHANGE_VAL 10
#endif

// forward declarations
int get_distance(int32_t *distance_mm);
int set_backlight_pwm(uint32_t duty_cycle);

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
        global_backlight_duty_cycle -= BACKLIGHT_CHANGE_VAL;
        // reset to max if min is exceeded
        if (global_backlight_duty_cycle < BACKLIGHT_MIN_VAL) {
            global_backlight_duty_cycle = BACKLIGHT_MAX_VAL;
        }
        ret = set_backlight_pwm(global_backlight_duty_cycle);
        if (ret != 0) {
            printk("ERROR setting PWM duty cycle: %d\r\n", ret);
            return;
        }
    }
}

//////
int set_backlight_pwm(uint32_t duty_cycle) {
    int ret;
    uint32_t backlight_period;

    backlight_period = (uint32_t)(backlight_pwm.period * (duty_cycle/100.0));
    ret = pwm_set_dt(&backlight_pwm, backlight_pwm.period, backlight_period);
    if (ret) {
        printk("Error %d: failed to set pulse width\n", ret);
        return -1;
    }
    printk("Setting backlight pulse period to %d / %d ns\n", backlight_period, backlight_pwm.period);
    return 0;
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
    const struct device *display;
    lv_obj_t *unit_label;
    lv_obj_t *measurement_label;
    lv_style_t measurement_label_style;

    // Initialize work items
    k_work_init_delayable(&button_work, sensor_work_handler);
    k_work_init_delayable(&backlight_button_work, backlight_work_handler);

    // initialize sensor
    if (!device_is_ready(vl53l1x)) {
        printk("ERROR: device is not ready: %s\n", vl53l1x->name);
        return 0;
    }

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

    // Initialize the display
    display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display)) {
        printk("Error: display not ready\r\n");
        return 0;
    }

    // create units label
    unit_label = lv_label_create(lv_scr_act());
    lv_label_set_text(unit_label, "mm");
    lv_obj_align(unit_label, LV_ALIGN_RIGHT_MID, -35, 5);
#ifdef CONFIG_BOARD_M5STICKC_PLUS2
    lv_obj_align(unit_label, LV_ALIGN_RIGHT_MID, -25, 5);
#endif

    // Adjust style for measurement label
	lv_style_init(&measurement_label_style);
	lv_style_set_text_font(&measurement_label_style, &lv_font_montserrat_20);

    // create measurement label
    measurement_label = lv_label_create(lv_scr_act());
    lv_obj_add_style(measurement_label, &measurement_label_style, 0);
    lv_label_set_text(measurement_label, "0");
    lv_obj_align(measurement_label, LV_ALIGN_CENTER, 0, 0);

    // Disable display blanking
    display_blanking_off(display);

    // Make sure backlight pwm was initialized
    if (!pwm_is_ready_dt(&backlight_pwm)) {
		printk("PWM is not ready\r\n");
		return 0;
	}

    // set backlight pwm pulse width
    global_backlight_duty_cycle = BACKLIGHT_MIN_VAL;
    ret = set_backlight_pwm(global_backlight_duty_cycle);
    if (ret != 0) {
        printk("ERROR setting PWM duty cycle: %d\r\n", ret);
		return 0;
    }

    while (1) {
        // update the measurement if required
        distance = get_latest_distance_nonblocking();
        if (distance >= 0) {
            sprintf(buf, "%d", distance);
            lv_label_set_text(measurement_label, buf);
        }


        // Must be called periodically
		lv_task_handler();

        k_msleep(sleep_time_ms);
    }

    return 0;
}