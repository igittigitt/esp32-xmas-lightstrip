# Bekannte Probleme im Code

Festgestellt bei Analyse am 2026-11-07 (Commit `35eb0a0` + uncommitted Änderungen). Nur beheben, wenn beauftragt;
nach Fix Eintrag entfernen oder als erledigt markieren.

## Kritisch

| # | Ort | Problem |
|---|---|---|
| K1 | `app_main()` | Endlos-Demo-Schleife (`led_on/led_blink/led_fade`) direkt nach `led_controller()`. Gesamter folgender Init-Code (I2C, Button, Sleep, Timer, Tasks) ist **unerreichbar**. |
| K2 | `save_timers_to_eeprom()` | Bei fehlender Signatur ruft sich die Funktion selbst auf, ohne die Signatur zu schreiben ⇒ **Endlosrekursion / Stack Overflow**. `load_timers_from_eeprom()` triggert das ebenfalls. |
| K3 | `sleep_task()` | Gibt `sleep_semaphore` am Schleifenende selbst wieder frei ⇒ Gerät legt sich ~100 ms nach jedem Wakeup sofort wieder schlafen; Neustart des `inactivity_timer` ist auskommentiert. |

## Mittel

| # | Ort | Problem |
|---|---|---|
| M1 | State Machine | Alle LED-Aktionen auskommentiert (`//TODO:`); DS3231-Alarm löst keine Aktion aus (nur Log). |
| M2 | `debug_task` | Weckt alle 5 s und macht I2C – auch wenn DS3231 im Sleep stromlos ist (`ds3231_power_off`). |
| M3 | Helligkeitseinheit | Uneinheitlich 0–255 vs. % (`edit_timer_interactive` 0–100, Ausgaben mit `%`). |
| M4 | `show_timers()` | Iteriert `i < count` und nimmt kompakte Liste an; `add_timer()` füllt beliebige freie Slots. |
| M5 | `led_controller_task` FADE/PULSE | uint8-Über-/Unterlauf von `fadeDuty` möglich; FLASH==BLINK, PULSE==FADE; `count` ignoriert. |
| M6 | `led_on(0)` | Wird zu voller Helligkeit (255). |
| M7 | GPIO5 | Taster active HIGH, aber `check_console_mode_on_boot()` aktiviert Pull-up ⇒ Taster gilt beim Boot evtl. immer als gedrückt / Dauerstrom. |
| M8 | Sleep + LEDC | LED aus ⇒ LEDC läuft weiter auf RC_FAST ⇒ unnötiger Sleep-Strom. |

## Gering

| # | Ort | Problem |
|---|---|---|
| G1 | `gpio_event_task`, `app_main` | `vTaskDelay(pdTICKS_TO_MS(100))` – falsches Makro, gemeint `pdMS_TO_TICKS`. |
| G2 | `set_ds3231_alarm()` | Format-String `"%03d%"` ungültig. |
| G3 | Konsolenmenü Option 8 | `buf[0] == 'y' \|\| buf[0] == 'y'` (gemeint `'Y'`). |
| G4 | `DS3231_MODULE_POWERSAVE` | definiert, nie ausgewertet. |
| G5 | `components/led_controller/CMakeLists.txt` | Kommentar nennt falschen Pfad (`at24c`). |
| G6 | `on_state_exit_program()` | Off-Timer-Überlauf um Mitternacht korrekt, aber Off-Timer kann mit bestehendem Timer kollidieren (keine Duplikatprüfung). |
| G7 | Git | `components/at24c/include/at24c.h` gelöscht, Header liegt jetzt im Komponenten-Root (uncommitted). |
