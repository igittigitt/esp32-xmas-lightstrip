# Architektur

## 1. Zielbild [ZIEL]

```
 ┌──────────────────┐   Zigbee (IEEE 802.15.4)   ┌──────────────────────────┐
 │ Home Assistant   │◄──────────────────────────►│ ESP32-H2 (Zigbee End Dev.)│
 │ ZHA / Zigbee2MQTT│   On/Off, Level, Battery   │                          │
 └──────────────────┘                            │  zigbee task ──► led ctrl ├──PWM──► KSQ (~100 mA) ──► LED-Kette
                                                 │       ▲             ▲     │
                                                 │  power monitor ─────┘     │
                                                 └───────────┬──────────────┘
                                                             │ gemeinsame Versorgung
                                                       ┌─────┴─────┐
                                                       │18650 1S + │
                                                       │Buck-Boost │
                                                       └───────────┘
```

- Home Assistant ist **die einzige Steuerinstanz** im Betrieb (An/Aus, Helligkeit; Zeitpläne als HA-Automationen).
- Das Gerät meldet Zustand (On/Off, Level) und Akkustand (Power Configuration Cluster) zurück.
- Das Gerät verbringt den größten Teil der Zeit im **Light Sleep**; die LED-PWM läuft dabei weiter.
- Lokaler Taster bleibt optional als Fallback/Pairing-Taste [OFFEN].

Details: [zigbee-integration.md](zigbee-integration.md), [power-management.md](power-management.md).

## 2. Ist-Architektur [IST]

Monolithische Applikation in [main/main.c](../main/main.c) plus zwei Komponenten.

### FreeRTOS-Tasks und Timer

| Name | Quelle | Aufgabe | Weckt periodisch? |
|---|---|---|---|
| `led controller` | `led_controller_task()` | Setzt LEDC-Duty je nach Modus, wartet auf `ledQueue` | Nur in Effekt-Modi (Blink/Fade) |
| `sleep` | `sleep_task()` | Wartet auf `sleep_semaphore`, ruft `esp_light_sleep_start()` (manueller Sleep) | nein |
| `gpio_event` | `gpio_event_task()` | Loggt GPIO-Interrupts (DS3231-Alarm) aus `gpio_evt_queue` | nein |
| `debug_task` | `debug_task()` (`#if DEBUG`) | Liest alle 5 s RTC + Alarm-Flags per I2C | **ja – verhindert Sleep-Effizienz** |
| `inactivity` (SW-Timer) | `inactivity_timer_callback()` | Nach 15 s ohne Tastendruck → `sleep_semaphore` | one-shot |
| `reset_timer` (SW-Timer) | `reset_timer_callback()` | 5 s Long-Press in RESET → alle Timer löschen | one-shot |
| iot_button intern | `espressif/button` | Entprellung, Klick-Gesten | ja, während aktiv; wird vor Sleep gestoppt |

### Datenfluss heute

```
Taster (GPIO5) ─► iot_button ─► on_press_repeat_done/on_long_press_* ─► state_machine_handle_button_event()
                                         │                                         │
                                         └─ xTimerReset(inactivity_timer)          ├─► Timer-Liste (RAM) ─► AT24C (EEPROM)
                                                                                   └─► DS3231 Alarm 2 setzen
DS3231 SQW (GPIO14, LOW aktiv) ─► GPIO-ISR ─► gpio_evt_queue ─► gpio_event_task (nur Log)
inactivity_timer ─► sleep_semaphore ─► sleep_task ─► esp_light_sleep_start()
```

LED-Ansteuerung aus der State Machine ist überall noch als `//TODO:led_*` auskommentiert – die State Machine
verändert die LED aktuell **nicht**. Der neue `led_controller` wird nur von der Demo-Schleife in `app_main()` genutzt.

### Komponenten

| Komponente | Rolle | Doku |
|---|---|---|
| `components/led_controller` | Asynchroner LED-Treiber (Queue-API, Gamma, Effekte) | [led-controller.md](led-controller.md) |
| `components/at24c` | EEPROM-Treiber mit Signatur `"XMAS"` | [timer-scheduler.md](timer-scheduler.md) |
| `espressif/button` ^3.3 | Klick-Gesten | [state-machine.md](state-machine.md) |
| `esp-idf-lib/ds3231`, `i2cdev` | RTC (alte I2C-API) | [hardware.md](hardware.md) |

## 3. Migration Ist → Ziel

| Ist-Baustein | Rolle im Ziel | Bemerkung |
|---|---|---|
| `led_controller` | **bleibt**, wird von Zigbee-Callbacks angesteuert | Level 0–254 → Duty 0–255 mappen; Fade für Level-Transitions nutzen |
| Light-Sleep-Logik | **ersetzt** durch Automatic Light Sleep + Zigbee-Sleep-Signal | manueller `sleep_task` passt nicht zum Zigbee-Stack |
| Button-State-Machine | stark reduziert [OFFEN] | z. B. Pairing / Factory Reset / lokales Toggle |
| DS3231 + Timer-Scheduler + AT24C | voraussichtlich **entbehrlich** [OFFEN] | Zeitpläne in HA; Zigbee hat Time-Cluster. Entfernen spart Bauteile + Strom |
| UART-Konsolenmenü | Debug-Werkzeug | im Release deaktivieren |
| NVS | wird für Zigbee benötigt (`zb_storage`), ggf. auch für letzten LED-Zustand | Partitionstabelle muss angepasst werden |

Empfohlene neue Komponenten: `zigbee_light` (Stack, Endpoints, Attribut-Callbacks), `power_monitor`
(Akkuspannung, Unterspannungsschutz), ggf. `app_state` (zentraler Zustand On/Off/Level, persistiert).
