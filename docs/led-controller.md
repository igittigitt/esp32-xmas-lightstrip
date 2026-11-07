# Komponente `led_controller`

Pfad: [components/led_controller/](../components/led_controller/) · Status: **[IST]**, ersetzt die auskommentierten `led_*`-Funktionen in `main.c`.

## 1. Konzept

- Ein eigener FreeRTOS-Task (`led_controller_task`, Prio 1, 3 KB Stack) besitzt die LEDC-Hardware exklusiv.
- Steuerung ausschließlich über **`ledQueue`** (Länge 1, `xQueueOverwrite`) ⇒ neuester Befehl gewinnt, Aufrufer blockiert nie.
- Task wartet mit `xQueueReceive(..., waitTicks)`: in statischen Modi `portMAX_DELAY` (**kein Wakeup – sleep-freundlich**),
  in Effekt-Modi bis zum nächsten Schritt (Deadline-basiert über `nextChange`, mit Catch-up-Schleife).
- **Gamma-Korrektur** 2.2 über LUT `gamma8[256]`: Eingabe-Duty 0–255 ist „wahrgenommene Helligkeit“.
- LEDC: LOW_SPEED, TIMER_0, CHANNEL_0, 8 Bit, 1 kHz, **`LEDC_USE_RC_FAST_CLK`** ⇒ PWM läuft im Light Sleep weiter.

## 2. API ([led_controller.h](../components/led_controller/led_controller.h))

```c
esp_err_t led_controller(gpio_num_t pin);         // Init: Gamma-LUT, LEDC, Queue, Task
esp_err_t led_set_mode(const LedConfig_t *cfg);   // generisch
esp_err_t led_off(void);
esp_err_t led_on(uint8_t duty);                   // duty 0 ⇒ wird zu 255!
esp_err_t led_blink(uint8_t duty1, uint8_t duty2, uint32_t period_ms);
esp_err_t led_fade(uint8_t duty1, uint8_t duty2, uint32_t period_ms);
esp_err_t led_pulse(uint8_t duty1, uint8_t duty2, uint32_t period_ms);
```

`LedConfig_t`: `mode`, `duty1`, `duty2`, `period1_ms`, `period2_ms`, `count` (−1 = endlos).

## 3. Modi – tatsächliches Verhalten

| Modus | Verhalten im Code | Wakeups |
|---|---|---|
| `LED_MODE_OFF` | Duty 0 | keine |
| `LED_MODE_ON` | `gamma8[duty1]`, `duty1==0` ⇒ 255 | keine |
| `LED_MODE_BLINK` | Wechsel duty1 (period1) / duty2 (period2, sonst period1) | je Flanke |
| `LED_MODE_FLASH` | **identisch zu BLINK** | je Flanke |
| `LED_MODE_FADE` | Rampe duty1↔duty2 in 5er-Schritten, **endlos hin und zurück** (kein One-shot) | je Schritt |
| `LED_MODE_PULSE` | **identisch zu FADE** | je Schritt |

`count` wird derzeit **ignoriert**. `duty2==0` bei FADE/PULSE ⇒ Ende = 255.

## 4. Bekannte Schwächen / Erweiterungsbedarf

- `fadeDuty` (uint8) kann über-/unterlaufen, wenn `duty1`/`duty2` keine Vielfachen von 5 im passenden Abstand sind.
- Kein One-shot-Fade ⇒ für Zigbee-Level-Transitions und Soft-Start nötig (`count` implementieren oder Modus `FADE_TO`).
- Kein Abfragen des aktuellen Zustands (für Zigbee-Reporting nötig).
- Bei OFF wird LEDC nicht gestoppt ⇒ RC_FAST bleibt im Sleep aktiv (Energie, siehe [power-management.md](power-management.md) §3).
- Alternative: LEDC-Hardware-Fade (`ledc_fade_func_install`, `ledc_set_fade_time_and_start`) spart CPU-Wakeups.
- `CMakeLists.txt` hat einen falschen Pfad-Kommentar; `REQUIRES driver` könnte auf `esp_driver_ledc esp_driver_gpio` präzisiert werden.
