# Powermanagement & Energieeffizienz

> **Priorität 1 des Projekts.** ESP32-H2, Funk und LED-Kette teilen sich eine Batterie/einen Akku.
> Jede Änderung muss ihren Effekt auf mittleren Strom und Akkulaufzeit berücksichtigen.

## 1. Energiebilanz – wo der Strom hingeht

| Betriebszustand | Dominanter Verbraucher | Optimierungshebel |
|---|---|---|
| LEDs an | **LED-Kette via KSQ (~100 mA bei 100 %)** | Helligkeit/Gamma, PWM-Frequenz, KSQ-Wirkungsgrad, Versorgungspfad der KSQ |
| LEDs aus, Gerät wartet | **ESP32-H2 Sleep-Strom + Funk-Polls + Leckströme** | Light Sleep, seltene Wakeups, Peripherie abschalten, keine Dauer-Spannungsteiler |
| Aktiv (Zigbee-RX/TX, I2C) | ESP CPU + Radio | Aktivphasen kurz halten, Arbeit bündeln |

Versorgung (siehe [hardware.md](hardware.md) §4): **18650 Li-Ion 1S (3000–5000 mAh)** → **Buck-Boost 3,3 V** → ESP32-H2;
LED über **Konstantstromquelle (~100 mA)**. Volle Helligkeit ⇒ ca. 30–50 h Leuchtdauer pro Ladung.

Folgerung: Bei **LED aus** entscheidet der Ruhestrom über die Laufzeit (Wochen vs. Tage).
Bei **LED an** ist ESP-Optimierung zweitrangig, aber Spannungsstabilität wird kritisch (siehe §5).
Konkrete Stromwerte immer gegen das ESP32-H2-Datenblatt und Messung (z. B. PPK2/µCurrent) verifizieren – keine Annahmen hart codieren.

## 2. Sleep-Modell

### [IST] Manueller Light Sleep
- `CONFIG_PM_ENABLE=y` in `sdkconfig.defaults`; im generierten sdkconfig zusätzlich
  `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`, `CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP=y`, `CONFIG_PM_SLP_DISABLE_GPIO=y`.
- `inactivity_timer` (15 s, `INACTIVITY_TIMEOUT_MS`) → `sleep_semaphore` → `sleep_task()`:
  1. UART leeren (`uart_wait_tx_idle_polling`)
  2. `iot_button_stop()` (Button-Polling-Timer anhalten)
  3. `ds3231_power_off()` (RTC läuft auf VBAT weiter)
  4. `esp_light_sleep_start()`
  5. nach Wakeup: `ds3231_power_on()` (+100 ms Stabilisierung), `iot_button_resume()`
- Wakeup-Quellen (GPIO-Level-Wakeup):
  - Taster `GPIO5` HIGH (`gpio_wakeup_enable(BUTTON_GPIO, GPIO_INTR_HIGH_LEVEL)`)
  - DS3231 SQW/INT `GPIO14` LOW (Alarm)
- Debug: `SLEEP_INDICATOR_GPIO` (GPIO4) = HIGH während Sleep → mit Oszilloskop/Logic-Analyzer messbar.
- LEDC nutzt `LEDC_USE_RC_FAST_CLK`, damit die PWM **im Light Sleep weiterläuft** (LED bleibt an, CPU schläft).

### [ZIEL] Automatic Light Sleep mit Zigbee
- Kein manueller `sleep_task` mehr. Das System schläft automatisch, wenn alle Tasks blockieren
  (Tickless Idle). Der Zigbee-Stack meldet Schlafbereitschaft (`ESP_ZB_COMMON_SIGNAL_CAN_SLEEP`) und
  ruft `esp_zb_sleep_now()` auf – siehe [zigbee-integration.md](zigbee-integration.md).
- `esp_pm_configure()` mit `light_sleep_enable = true`, `max_freq_mhz` = 96 (H2-Maximum), `min_freq_mhz` = niedrig (z. B. 32) [OFFEN: messen].
- Wakeups pro Stunde minimieren: Zigbee-Poll-Intervall ist der Haupt-Trade-off (Reaktionszeit vs. Strom).

### Deep Sleep – nicht geeignet als Hauptmodus
LEDC/PWM stoppt im Deep Sleep, Zigbee-Netzwerkzustand muss neu aufgebaut werden (Rejoin kostet Energie).
Allenfalls sinnvoll als **Unterspannungs-Notabschaltung** (LED aus, langer Schlaf, keine Funkaktivität).

## 3. Regeln für Code

1. **Blockierend warten** (`portMAX_DELAY` oder exakter nächster Zeitpunkt). Vorbild: `led_controller_task()` setzt
   `waitTicks = portMAX_DELAY` in statischen Modi (OFF/ON).
2. **Keine periodischen Debug-/Status-Tasks im Release** (`debug_task` weckt alle 5 s und macht I2C).
3. **LED aus ⇒ LEDC stoppen** (`ledc_stop()` mit Idle-Level 0, ggf. `ledc_timer_pause()`): Solange LEDC
   auf RC_FAST läuft, bleibt RC_FAST im Sleep an und erhöht den Ruhestrom. **LED an** ⇒ RC_FAST nötig.
4. **Effekt-Modi (Blink/Fade/Pulse) wecken die CPU bei jedem Schritt** (`led_controller_task`). Nur kurzzeitig
   verwenden (Feedback, Transitions), nie als Dauerzustand. Alternative für Fades: Hardware-Fade
   (`ledc_set_fade_with_time` / `ledc_fade_start`) – prüfen, ob es im Light Sleep mit RC_FAST funktioniert [OFFEN].
5. **Peripherie schaltbar bestromen** (Muster `DS3231_VCC_PIN`). Gilt auch für künftigen Akku-Spannungsteiler.
6. **GPIO-Zustände im Sleep definieren**: Ausgänge auf definierten Pegel, keine floatenden Eingänge, interne
   Pull-ups nur wo nötig (jeder Pull-up gegen aktiven Pegel = Dauerstrom). `CONFIG_PM_SLP_DISABLE_GPIO` beachten –
   die PWM-Pin-Konfiguration darf dabei nicht verloren gehen (testen!).
7. **Logging reduzieren**: UART-TX hält das System wach; im Release `LOG_LEVEL` auf WARN/ERROR, `DEBUG 0`.
8. **I2C nur gebündelt**, danach Gerät wieder abschalten.
9. **PM-Locks** (`esp_pm_lock_acquire`) nur um zeitkritische Abschnitte, immer paarweise freigeben.

## 4. Akkuüberwachung [ZIEL]

- Akkuspannung per ADC über **schaltbaren** Spannungsteiler (GPIO oder MOSFET schaltet Teiler nur zur Messung ein)
  oder hochohmigen Teiler + Kondensator. Messung selten (z. B. alle 10–60 min) und gebündelt mit ohnehin nötigem Wakeup.
- Messung **bei LED aus oder mit kurz definierter Last**, da Spannung unter LED-Last einbricht (Innenwiderstand).
- Meldung an HA über Zigbee **Power Configuration Cluster (0x0001)**: `BatteryVoltage` (Einheit 100 mV),
  `BatteryPercentageRemaining` (Einheit 0,5 %). Reporting-Intervall lang wählen.
- **Unterspannungsstrategie für Li-Ion 1S** (Leerlaufspannung, bei LED aus gemessen; Werte als Startpunkt, anpassbar):

  | Schwelle | Spannung | Aktion |
  |---|---|---|
  | Voll | 4,2 V | 100 % |
  | Warnung | ≈ 3,5 V | Helligkeit begrenzen, Low-Battery an HA melden |
  | LED-Abschaltung | ≈ 3,3 V | LED aus, LED-Befehle aus HA ablehnen/ignorieren, nur minimaler Funkbetrieb |
  | Kritisch | ≈ 3,1 V | Deep Sleep ohne Funk und ohne Timer-Wakeup (bzw. sehr selten zur Kontrolle) |

- **Warum Software-Schutz Pflicht ist:** Der Buck-Boost arbeitet bis **2,0 V** Eingang – er würde die Zelle
  weit unter 3,0 V tiefentladen und dauerhaft schädigen. Eine Zellschutzschaltung (falls vorhanden, typ. ~2,5 V)
  ist nur letzte Absicherung. Zusätzlich Hysterese einbauen (Wiedereinschalten erst nach Laden, z. B. > 3,7 V).
- Prozentkurve nicht linear: Li-Ion-Entladekurve als Lookup-Tabelle abbilden.
- Ruhestrom des Spannungsteilers: z. B. 1 MΩ gesamt ⇒ 4 µA dauerhaft – relevant gegenüber dem Sleep-Strom; daher schaltbar.

## 5. Gemeinsame Versorgung von ESP und LED-Kette

Risiken, weil LED-Last und ESP am selben Akku hängen:
- **Spannungseinbruch beim Einschalten** (Einschaltstrom, Akku-Innenwiderstand) → Brownout-Reset des ESP.
  Gegenmaßnahmen: LED immer per Fade einschalten (Soft-Start), ausreichende Pufferkondensatoren am ESP-Regler,
  Brownout-Detector aktiv lassen.
- **PWM-Ripple** auf der Versorgung → Funkstörungen/ADC-Fehler. PWM-Frequenz (aktuell 1 kHz, 8 Bit) und
  Entkopplung beachten; ADC-Messung nicht während PWM-Flanken [OFFEN: synchronisieren oder bei LED aus messen].
- **Buck-Boost-Wandler für ESP**: Ruhestrom (Iq) und Wirkungsgrad bei sehr kleiner Last können den Sleep-Strom
  des H2 übertreffen. Wandler mit PFM/Power-Save-Modus und Iq im µA-Bereich bevorzugen [OFFEN: Typ].
  Bei Akkuspannung um 3,3 V wechselt der Wandler zwischen Buck- und Boost-Betrieb – Ripple/Übergang prüfen.
- **KSQ-Versorgungspfad**: Hängt die KSQ am 3,3-V-Ausgang, laufen die ~100 mA LED-Strom durch den Wandler
  (Wirkungsgradverlust, Lastsprünge direkt auf der ESP-Versorgung). Direkt am Akku ist energetisch und für die
  Stabilität meist günstiger [OFFEN: aktuelle Verdrahtung].

## 6. Offene Punkte

- [OFFEN] Exakter Buck-Boost-Typ und dessen Ruhestrom.
- [OFFEN] KSQ: Versorgung (Akku vs. 3,3 V), Dimm-Eingang, PWM-Frequenzbereich.
- [OFFEN] Gemessener Sleep-Strom Ist-Zustand.
- [OFFEN] Ziel-Laufzeit (z. B. Saison Nov–Jan mit X h/Tag an).
