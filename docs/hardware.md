# Hardware

## 1. SoC

- **ESP32-H2** (`CONFIG_IDF_TARGET="esp32h2"`), RISC-V, max. 96 MHz, IEEE 802.15.4 + BLE, kein WLAN.
- Flash 4 MB (`CONFIG_ESPTOOLPY_FLASHSIZE_4MB`).
- Konsole: UART0 @115200 (USB-Serial-JTAG als Konsole deaktiviert; `reinit_usb_cdc()` existiert, wird nicht genutzt).

## 2. Pinbelegung [IST] (Quelle: `#define`s in [main/main.c](../main/main.c))

| GPIO | Define | Richtung | Funktion |
|---|---|---|---|
| 13 | `LED_GPIO` | Out (LEDC PWM) | LED-Kette, 1 kHz, 8 Bit, LEDC_TIMER_0/CHANNEL_0 |
| 5 | `BUTTON_GPIO` | In | Taster, **active HIGH** (`BUTTON_ACTIVE_LEVEL 1`); Wakeup HIGH; beim Boot gedrückt ⇒ Konsolenmodus |
| 14 | `DS3231_ALARM_INT_PIN` | In, Pull-up, NEGEDGE-ISR | DS3231 SQW/INT, **LOW = Alarm**; Wakeup LOW |
| 10 | `DS3231_VCC_PIN` | Out | Schaltet VCC des DS3231-Moduls (HIGH = an). Aus ⇒ RTC läuft auf VBAT |
| 11 | `I2C_MASTER_SCL_IO` | I2C | SCL, 100 kHz, I2C_NUM_0 |
| 12 | `I2C_MASTER_SDA_IO` | I2C | SDA |
| 4 | `SLEEP_INDICATOR_GPIO` | Out | Debug: HIGH während Light Sleep |

Hinweis: Kommentar in `button_init()` spricht von „Active LOW“, der Code nutzt aber active HIGH – Hardware entscheidet [OFFEN: Beschaltung prüfen, Pull-down vorhanden?]. `check_console_mode_on_boot()` aktiviert einen **internen Pull-up** auf GPIO5, was bei active-HIGH-Taster Dauerstrom/Fehlauslösung bedeuten kann.

## 3. I2C-Geräte (DS3231-Modul, z. B. „ZS-042“)

| Gerät | Adresse | Bemerkung |
|---|---|---|
| DS3231 RTC | 0x68 | Alarm 2 (Stunde+Minute) für Timer; `BBSQW`-Bit gesetzt, damit INT auch auf VBAT funktioniert |
| AT24C32 EEPROM | 0x57 | 4096 Byte, 32-Byte-Pages, 16-Bit-Adressen, 10 ms Write-Cycle |

Treiber: `esp-idf-lib/ds3231` + `i2cdev` (Legacy-I2C-API). Hinweis auf Module dieser Bauart: Lade-Schaltung für
VBAT (Diode+Widerstand) kann eine nicht wiederaufladbare CR2032 schädigen [OFFEN: Modul modifiziert?].

Im Zigbee-Zielbild ist das Modul voraussichtlich entbehrlich (siehe [architecture.md](architecture.md) §3).

## 4. Stückliste Versorgung & LED [IST – Angabe Nutzer]

| Baugruppe | Spezifikation | Konsequenz |
|---|---|---|
| Akku | **Li-Ion 18650, 1S**, 3000–5000 mAh (4,2 V voll, ~3,6–3,7 V nominal) | Tiefentladeschutz zwingend, siehe [power-management.md](power-management.md) §4 |
| LED-Treiber | **KSQ-Shield (Konstantstromquelle), ~100 mA** | Strom ist durch KSQ begrenzt; Helligkeit per PWM an GPIO13 auf den Enable/Dim-Eingang [OFFEN: Eingangstyp] |
| Spannungswandler | **Buck-Boost**, Eingang 2,0–5,0 V, Ausgang 3,3 V, 0,5–1 A | ESP läuft über den ganzen Akku-Bereich – auch **unterhalb** der sicheren Li-Ion-Grenze ⇒ Software-Abschaltung nötig |
| SoC | ESP32-H2 | |

Grobe Laufzeitabschätzung (nur LED, volle Helligkeit, KSQ direkt am Akku, Wandlerverluste ignoriert):
3000 mAh / 100 mA ≈ **30 h**, 5000 mAh ≈ **50 h**. Bei z. B. 6 h/Tag ⇒ ca. 5–8 Tage. Dimmen verlängert linear
zum mittleren Duty (vor Gamma: wahrgenommene Helligkeit ≠ Strom!). Im LED-aus-Zustand bestimmen
Buck-Boost-Ruhestrom + ESP-Sleep + Zigbee-Polls die Laufzeit.

### Offene Punkte
- [OFFEN] Wird die KSQ direkt aus dem Akku oder aus 3,3 V des Buck-Boost gespeist? (Bei 3,3 V: LED-Last läuft über den Wandler ⇒ Wandlerwirkungsgrad und 0,5–1 A-Limit relevant, ESP-Versorgung wird durch LED-Lastsprünge gestört.)
- [OFFEN] Konkreter Buck-Boost-Typ: **Ruhestrom (Iq)**, Wirkungsgrad bei µA-/mA-Last, Enable-Pin, Power-Save-/PFM-Modus.
- [OFFEN] KSQ-Shield: Dimm-Eingang (PWM-Logikpegel? max. PWM-Frequenz?), Verhalten bei Eingangsspannung < LED-Flussspannung + Dropout (sinkender Akku ⇒ sinkende Helligkeit).
- [OFFEN] Hat die 18650 eine Schutzschaltung (protected cell)? Ladeschaltung vorhanden (z. B. TP4056)?
- [OFFEN] Pufferkondensatoren, Brownout-Verhalten.
- [OFFEN] ADC-Pin für Akkuspannung (ESP32-H2: ADC1 auf GPIO1–GPIO5; GPIO5 ist durch Taster belegt). 4,2 V übersteigt den ADC-Bereich ⇒ Spannungsteiler (schaltbar oder hochohmig).
