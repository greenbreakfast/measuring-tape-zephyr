include(${ZEPHYR_BASE}/boards/common/esp32.board.cmake)

# ESP32-PICO-V3-02 specific settings
board_runner_args(esp32 
    "--esp-flash-size=8MB"
    "--esp-flash-mode=dio"  # PICO modules typically use DIO
    "--esp-flash-freq=40m"
)