# AGENTS.md – ESP32 Xmas Lightstrip Controller

Leitfaden für KI-Agenten (Claude Code, Codex, Copilot …), die an diesem Repository arbeiten.
Vor jeder nicht-trivialen Änderung die passenden Dokumente unter [docs/](docs/) lesen.

## 1. Projektziel (Zielbild)

Akkubetriebene Steuerung für eine LED-Lichterkette auf Basis **ESP32-H2**.

- **Einzige Kommunikationsschnittstelle im Betrieb: Zigbee** zu einem **Home Assistant** (ZHA oder Zigbee2MQTT).
- Funktionsumfang primär: **An/Aus** und **Helligkeit** (Zigbee On/Off + Level Control).
- ESP32 und LED-Kette hängen an **derselben Batterie/Akku** → **Powermanagement ist Kernanforderung**.
- Hardware: **ESP32-H2**, **Li-Ion 18650 (1S, 3000–5000 mAh)**, **LED-KSQ-Shield (~100 mA)**,
  **Buck-Boost 2,0–5,0 V → 3,3 V (0,5–1 A)**. Da der Wandler bis 2,0 V arbeitet, muss die Firmware
  den Akku vor Tiefentladung schützen ([docs/power-management.md](docs/power-management.md) §4).
- **Energieeffizienz hat oberste Priorität.** Jede Designentscheidung wird zuerst danach bewertet,
  wie sie sich auf den mittleren Stromverbrauch und die Akkulaufzeit auswirkt.

## 2. Ist-Zustand (Stand: Commit `35eb0a0` + uncommitted Änderungen)

Der aktuelle Code ist ein **Lern-/Vorgängerstand ohne Zigbee**:

| Bereich | Status |
|---|---|
| LED-PWM (LEDC, Gamma-Korrektur, Effekte) | implementiert in [components/led_controller/](components/led_controller/) |
| Light-Sleep + Inaktivitäts-Timer | implementiert in [main/main.c](main/main.c), fehlerhaft (siehe Known Issues) |
| Button-State-Machine (Klick-Gesten) | implementiert, LED-Feedback auskommentiert (`//TODO:`) |
| DS3231-RTC + Alarm-Wakeup, Timer-Scheduler | implementiert |
| AT24C32-EEPROM (Timer-Persistenz) | implementiert in [components/at24c/](components/at24c/) |
| UART-Konsolenmenü (Timer/Uhr setzen) | implementiert |
| **Zigbee / Home Assistant** | **nicht vorhanden – Hauptziel** |
| Akkuspannungsmessung / Unterspannungsschutz | nicht vorhanden |

**Achtung:** `app_main()` enthält aktuell eine Endlos-Demo-Schleife des LED-Controllers;
der gesamte nachfolgende Init-Code (I2C, Button, Sleep, Timer) ist dadurch unerreichbar.
Details: [docs/known-issues.md](docs/known-issues.md).

## 3. Dokumentation (docs/)

| Datei | Inhalt |
|---|---|
| [docs/README.md](docs/README.md) | Index, Glossar, Lesereihenfolge |
| [docs/architecture.md](docs/architecture.md) | Systemüberblick, Tasks, Komponenten, Ist vs. Ziel |
| [docs/power-management.md](docs/power-management.md) | **Energiekonzept**, Sleep-Modi, gemeinsame Versorgung, Regeln |
| [docs/zigbee-integration.md](docs/zigbee-integration.md) | Geplante Zigbee-Anbindung, Cluster, Sleepy End Device, HA |
| [docs/hardware.md](docs/hardware.md) | Pinbelegung, I2C-Geräte, Wakeup-Quellen, offene HW-Fragen |
| [docs/led-controller.md](docs/led-controller.md) | API und Interna der LED-Komponente |
| [docs/state-machine.md](docs/state-machine.md) | Button-Gesten und Zustandsautomat |
| [docs/timer-scheduler.md](docs/timer-scheduler.md) | RTC-Timer, EEPROM-Layout, Konsolenmenü |
| [docs/build-and-debug.md](docs/build-and-debug.md) | Toolchain, Build, Flash, Monitor, Debug-Hilfen |
| [docs/known-issues.md](docs/known-issues.md) | Bekannte Bugs und Inkonsistenzen im Code |

## 4. Repository-Struktur

```
main/main.c                     Applikation (Sleep, Button, State Machine, RTC, Timer, Konsole) – monolithisch
main/idf_component.yml          Registry-Abhängigkeiten (button, ds3231, i2cdev)
components/led_controller/      LED-PWM-Task mit Queue-API (LEDC, Gamma 2.2)
components/at24c/               Eigener AT24C-EEPROM-Treiber (auf i2cdev)
sdkconfig.defaults              Quelle der Wahrheit für Kconfig (sdkconfig ist generiert, gitignored)
state_machine.drawio            Entwurf der State Machine (weicht vom Code ab)
.devcontainer/                  ESP-IDF Docker-Umgebung
managed_components/, build/     generiert – nie bearbeiten, nie committen
```

## 5. Toolchain & Befehle

- **ESP-IDF v5.5.1**, Target **`esp32h2`** (RISC-V). Siehe [dependencies.lock](dependencies.lock).
- Build: `idf.py build` · Flash+Monitor: `idf.py -p <PORT> flash monitor`
- Target nur bei Bedarf setzen: `idf.py set-target esp32h2` (löscht sdkconfig!)
- Konsole läuft über **UART0 @115200** (nicht USB-Serial-JTAG), siehe `sdkconfig.defaults`.
- Es gibt keine automatisierten Tests. Verifikation = Build + Hardwaretest. Wenn kein Build möglich ist,
  das dem Nutzer explizit sagen statt Erfolg zu behaupten.

## 6. Verbindliche Regeln für Agenten

### Energieeffizienz (höchste Priorität)
1. **Kein Busy-Waiting, kein Polling.** Blockierend warten (`xQueueReceive`, `xSemaphoreTake`, Event-Groups)
   mit `portMAX_DELAY` oder dem nächsten tatsächlich benötigten Zeitpunkt.
2. **Keine periodischen Tasks ohne Notwendigkeit.** Jeder Tick-Wakeup verhindert Light-Sleep.
   Debug-Tasks nur hinter `#if DEBUG`.
3. **Peripherie nur bei Bedarf bestromen** (Muster: `ds3231_power_on/off()`). Spannungsteiler,
   Sensoren, Pull-ups nicht dauerhaft aktiv lassen.
4. **Automatic Light Sleep** (`CONFIG_PM_ENABLE` + `CONFIG_FREERTOS_USE_TICKLESS_IDLE`) ist das Zielmodell;
   Power-Management-Locks (`esp_pm_lock_*`) nur so kurz wie möglich halten.
5. Bei LED aus: LEDC-Timer anhalten, damit `RC_FAST_CLK` im Sleep abgeschaltet werden kann.
6. Neue Features immer mit einer kurzen Abschätzung „Wirkung auf Sleep-Strom / Wakeups pro Stunde“ begründen.
7. Kconfig-Änderungen, die Strom betreffen, in `sdkconfig.defaults` eintragen und in
   [docs/power-management.md](docs/power-management.md) dokumentieren.

### Code
- Sprache **C (C11)**, ESP-IDF-Stil, `esp_err_t`-Rückgaben, `ESP_LOGx` mit modulweitem `TAG`.
- Neue Funktionalität als **eigene Komponente unter `components/`** anlegen (z. B. `zigbee_light`,
  `power_monitor`) statt `main.c` weiter wachsen zu lassen.
- Kommunikation zwischen Tasks über Queues/Event-Groups (Vorbild: `ledQueue` mit `xQueueOverwrite`).
- Kein Aufruf von blockierendem I2C/Flash-Code aus ISRs oder Timer-Callbacks; an Tasks delegieren.
- Kommentare sind gemischt Deutsch/Englisch; innerhalb einer Datei konsistent bleiben.
- Auskommentierten Altcode nicht stillschweigend löschen oder reaktivieren – nur wenn die Aufgabe es verlangt.
- Bugs aus [docs/known-issues.md](docs/known-issues.md) nur beheben, wenn beauftragt; bei Fix den Eintrag aktualisieren.

### Hardware-Sicherheit
- GPIO-Belegung nicht ändern ohne Rückfrage – sie entspricht der realen Verdrahtung ([docs/hardware.md](docs/hardware.md)).
- LED-Last und ESP teilen die Versorgung: harte Helligkeitssprünge (0→100 %) vermeiden, Fades bevorzugen
  (Einbruch der Versorgungsspannung / Brownout).

### Git
- `build/`, `managed_components/`, `sdkconfig`, `sdkconfig.old` nie committen.
- Commits nur auf Anweisung des Nutzers.

## 7. Dokumentationspflege

Ändert sich ein Konzept (Pins, Sleep-Strategie, Zigbee-Cluster, EEPROM-Layout, State Machine),
das zugehörige Dokument in `docs/` **im selben Arbeitsschritt** aktualisieren.
Jedes Dokument kennzeichnet Aussagen mit **[IST]** (im Code vorhanden), **[ZIEL]** (geplant) oder **[OFFEN]** (ungeklärt).
