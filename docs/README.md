# Dokumentation – Index

Diese Dokumente sind für **KI-Agenten und Entwickler** geschrieben: knapp, strukturiert, mit eindeutigen
Bezeichnern aus dem Code. Einstieg für Agenten ist [../AGENTS.md](../AGENTS.md).

## Status-Markierungen

| Marker | Bedeutung |
|---|---|
| **[IST]** | Im aktuellen Code vorhanden (ggf. mit Bugs) |
| **[ZIEL]** | Geplant / Zielarchitektur, noch nicht implementiert |
| **[OFFEN]** | Ungeklärte Entscheidung oder fehlende Information – vor Implementierung mit dem Nutzer klären |

## Lesereihenfolge

1. [architecture.md](architecture.md) – Gesamtbild, Ist vs. Ziel
2. [power-management.md](power-management.md) – Energiekonzept (höchste Priorität)
3. [zigbee-integration.md](zigbee-integration.md) – Hauptziel der Weiterentwicklung
4. [hardware.md](hardware.md) – Pins und Beschaltung
5. Komponenten-Details: [led-controller.md](led-controller.md), [state-machine.md](state-machine.md), [timer-scheduler.md](timer-scheduler.md)
6. [build-and-debug.md](build-and-debug.md), [known-issues.md](known-issues.md)

## Glossar

| Begriff | Bedeutung |
|---|---|
| ESP32-H2 | Espressif-SoC, RISC-V, IEEE 802.15.4 (Zigbee/Thread) + BLE, **kein WLAN** |
| LEDC | LED-PWM-Peripherie des ESP32 |
| Duty | PWM-Tastverhältnis, hier 8 Bit (0–255) |
| Light Sleep | CPU angehalten/abgeschaltet, RAM bleibt erhalten, Wakeup per GPIO/Timer/Funk; Programm läuft danach weiter |
| Deep Sleep | Fast alles aus, Wakeup = Neustart; für dieses Projekt nur eingeschränkt nutzbar (LED-PWM stoppt) |
| PM / Auto Light Sleep | `CONFIG_PM_ENABLE` + Tickless Idle: System schläft automatisch, wenn alle Tasks blockieren |
| ZED / SED | Zigbee End Device / Sleepy End Device (schläft, pollt Parent zyklisch nach Nachrichten) |
| ZHA / Z2M | Home-Assistant-Zigbee-Integrationen (Zigbee Home Automation / Zigbee2MQTT) |
| DS3231 | I2C-Echtzeituhr mit Alarm-Ausgang (SQW/INT), auf dem Modul mit AT24C32-EEPROM |
| VBAT | Pufferbatterie des DS3231 (Knopfzelle), hält Uhr bei abgeschalteter VCC |
