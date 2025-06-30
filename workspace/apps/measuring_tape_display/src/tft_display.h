#ifndef TFT_DISPLAY_H_
#define TFT_DISPLAY_H_

// backlight defines based on device
#ifdef CONFIG_BOARD_M5STICKC_PLUS2
#define TFT_DISPLAY_BACKLIGHT_MAX_VAL 100
#define TFT_DISPLAY_BACKLIGHT_MIN_VAL 35
#define TFT_DISPLAY_BACKLIGHT_CHANGE_VAL 15
#else
#define TFT_DISPLAY_BACKLIGHT_MAX_VAL 45
#define TFT_DISPLAY_BACKLIGHT_MIN_VAL 5
#define TFT_DISPLAY_BACKLIGHT_CHANGE_VAL 10
#endif

int tft_display_init(void);
int tft_display_set_backlight_pwm(uint32_t duty_cycle);
int tft_display_set_measurement_text(const char *text);
void tft_display_task_handler(void);

#endif // TFT_DISPLAY_H_