# Timer-Scheduler, RTC & EEPROM-Persistenz [IST]

> Im Zigbee-Zielbild übernimmt Home Assistant die Zeitpläne. Dieses Subsystem ist dann voraussichtlich
> entbehrlich [OFFEN]. Vor Entfernen mit dem Nutzer klären.

## 1. Konzept

- Bis zu `MAX_TIMERS = 8` Schaltpunkte, je **Uhrzeit (hh:mm) + Helligkeit**. Ein „Programm“ besteht aus einem
  On-Timer (Helligkeit > 0) und einem Off-Timer (Helligkeit 0).
- Tagesperiodisch, Datum wird ignoriert. Zeitarithmetik in Minuten seit Mitternacht mit Überlauf
  (`time_to_minutes`, `minutes_until`, `minutes_since`).
- `get_previous_timer()` ⇒ aktuell gültige Helligkeit; `get_next_timer()` ⇒ nächster DS3231-Alarm.
- DS3231 **Alarm 2** (`DS3231_ALARM2_MATCH_MINHOUR`) zieht SQW/INT (GPIO14) auf LOW ⇒ Wakeup aus Light Sleep.

## 2. Datenstruktur

```c
typedef struct __attribute__((packed)) {
  uint8_t hour;        // 0-23
  uint8_t minute;      // 0-59
  uint8_t brightness;  // 0-255 (Duty) – Konsole/Logs teils fälschlich als %
  bool    active;      // Slot belegt
} led_timer_t;         // 4 Byte
static led_timer_t timers[MAX_TIMERS];
```

`add_timer()` belegt den ersten Slot mit `active == false`; Liste ist **nicht sortiert**.

## 3. EEPROM-Layout (AT24C32 @0x57)

| Adresse | Länge | Inhalt |
|---|---|---|
| 0x0000 | 4 | Signatur `"XMAS"` (`AT24C_SIGNATURE`, ohne Nullterminator) |
| 0x0004 (`DS3231_EEPROM_DATA_OFFSET`) | 8 × 4 = 32 | `timers[0..7]` (Rohbytes von `led_timer_t`) |

Format-Versionierung existiert nicht – bei Strukturänderung Signatur ändern oder Versionsbyte einführen.
EEPROM-Schreibzyklen sind begrenzt (~1 Mio.): nur bei Änderungen schreiben.

## 4. UART-Konsolenmenü

Aktivierung: Taster beim Booten gedrückt halten (`check_console_mode_on_boot`). Sleep ist dann deaktiviert;
Menü `timer_terminal_menu()`:

1 anzeigen · 2 hinzufügen · 3 bearbeiten · 4 löschen · 5 neu laden · 6 alle löschen ·
7 Uhrzeit/Datum setzen · 8 Alarm-Flags anzeigen/löschen · 0 Neustart.

Eingabe zeilenweise über `read_line()` (Polling mit 10 ms Delay – nur im Konsolenmodus akzeptabel).
