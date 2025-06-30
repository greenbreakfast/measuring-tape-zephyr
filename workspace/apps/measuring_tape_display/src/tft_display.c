#include <zephyr/kernel.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <string.h>

#include "tft_display.h"

static const struct pwm_dt_spec backlight_pwm = PWM_DT_SPEC_GET(DT_ALIAS(backlight_ctrl));
static lv_obj_t *measurement_label;

int tft_display_init(void) {
    const struct device *display;
    lv_obj_t *unit_label;
    lv_style_t measurement_label_style;

    // Initialize the display
    display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display)) {
        printk("Error: display not ready\r\n");
        return -1;
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
		return -1;
	}

    return 0;
}

// set display backlight based on the duty cycle percentage
// duty_cycle is in percentage (0-100)
int tft_display_set_backlight_pwm(uint32_t duty_cycle) {
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

int tft_display_set_measurement_text(const char *text) {
    if (measurement_label == NULL) {
        printk("Error: measurement label is not initialized\n");
        return -1;
    }

    // Set the text of the measurement label
    lv_label_set_text(measurement_label, text);
    return 0;
}

void tft_display_task_handler(void) {
    // Must be called periodically
	lv_task_handler();
}