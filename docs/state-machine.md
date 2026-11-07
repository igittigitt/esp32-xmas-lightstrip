# Button-State-Machine [IST]

Implementiert in [main/main.c](../main/main.c): `state_machine_handle_button_event()`, `state_machine_transition()`,
`on_state_enter_*` / `on_state_exit_*`. Entwurf: [state_machine.drawio](../state_machine.drawio) (**weicht vom Code ab**).

Im Zigbee-Zielbild wird diese State Machine stark reduziert (Pairing/Reset/Toggle) [OFFEN].

## 1. Events (aus `espressif/button`)

| Event | Quelle | Parameter |
|---|---|---|
| `BTN_EVENT_SINGLE_CLICK` / `DOUBLE_CLICK` / `TRIPLE_CLICK` | `BUTTON_PRESS_REPEAT_DONE` (einmal nach Abschluss der Klickserie) | `SHORT_PRESS_TIME_MS 300` |
| `BTN_EVENT_LONG_CLICK_START` | `BUTTON_LONG_PRESS_START` | `LONG_PRESS_TIME_MS 1000` |
| `BTN_EVENT_LONG_CLICK_HOLD` | `BUTTON_LONG_PRESS_HOLD` (wiederholt solange gedrückt) | |
| `BTN_EVENT_LONG_CLICK_END` | `BUTTON_LONG_PRESS_UP` | |

Jeder Button-Callback setzt `inactivity_timer` zurück (verzögert Sleep).

## 2. Zustände und Übergänge (Code)

| Zustand | Event | → Ziel | Aktion |
|---|---|---|---|
| `STATE_OFF` | SINGLE | `STATE_TIMER` | Timer-Betrieb starten |
| `STATE_OFF` | LONG_START | `STATE_RESET` | 5-s-`reset_timer` starten |
| `STATE_RESET` | LONG_END | `STATE_OFF` | Timer stoppen; nach ≥5 s Halten wurden alle Timer gelöscht (`delete_all_timers`) |
| `STATE_TIMER` | SINGLE | `STATE_OFF` | |
| `STATE_TIMER` | DOUBLE | `STATE_PROGRAM` | Startzeit = jetzt, Helligkeit 255 |
| `STATE_PROGRAM` | DOUBLE | `STATE_TIMER` | Exit: On-Timer (Startzeit, Helligkeit) + Off-Timer (jetzt, 0) anlegen, EEPROM speichern |
| `STATE_PROGRAM` | TRIPLE | `STATE_TIMER` | Exit: Abbruch, nichts speichern |
| `STATE_PROGRAM` | LONG_HOLD | – | Helligkeit ±3 pendelnd zwischen 10 und 255 (`led_brightness`, `led_dim_direction`) |
| `STATE_PROGRAM_DIM` | – | – | definiert, aber ungenutzt (Logik auskommentiert) |

`on_state_enter_timer()`: liest RTC, ermittelt vorherigen Timer (→ Soll-Helligkeit) und nächsten Timer (→ DS3231 Alarm 2).

## 3. Lücken

- Sämtliche LED-Rückmeldungen sind `//TODO:` – die LED reagiert nicht auf Zustände.
- Der DS3231-Alarm (Wakeup) wird nur geloggt (`gpio_event_task`); es gibt **keinen** Übergang „Alarm ⇒ Helligkeit setzen ⇒ nächsten Alarm setzen“.
- Zustand wird nicht persistiert (`// TODO: load initial state from NVS`).
- Drawio kennt zusätzlich `TIMER_PROGRAM_ON/OFF`, `LED_DIM`, `STATE_SLEEP`, `button_timeout(60s)`.
