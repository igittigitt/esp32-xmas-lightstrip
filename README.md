# Learn project "ESP32 Xmas Lightstrip Controller"

The purpose of this project is to get familar with ESP32(-H2) programming and coding in C++ using VS Code an Espressif-IDF.

# Changelog

- setup emtpy project
- added LED PWM control code to setup LEDC and set brightness
- added FreeRTOS component for multitasking
- added NVS component for Flash persistence
- added iot_button component for button "gestures" detection
- added state machine code
- added support for DS3231 RTC

# Business logic

- After first poweron, default values will be loaded (LED=OFF, Timer=NONE, Status=OFF)
- After powerloss/battery-change, last status restored from NVS
- In OFF mode press button 