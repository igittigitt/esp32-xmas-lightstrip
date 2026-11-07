# Zigbee-Integration mit Home Assistant [ZIEL]

> Noch **nicht implementiert**. Dieses Dokument beschreibt das Zielkonzept. Vor Implementierung die
> aktuelle Doku/Beispiele von `esp-zigbee-sdk` (Espressif) prüfen – API-Namen können sich zwischen Versionen ändern.

## 1. Anforderungen

- Zigbee ist im Betrieb die **einzige** Schnittstelle (kein WLAN – der ESP32-H2 hat keins; kein BLE im Betrieb).
- Funktionen: **An/Aus** und **Helligkeit**. Rückmeldung des Zustands und des **Akkustands** an HA.
- Batteriebetrieb ⇒ Gerät muss **Zigbee End Device (ZED)** sein, idealerweise **Sleepy End Device**.
  Ein Router (Standardrolle für Zigbee-Lampen) ist ausgeschlossen, da Router dauerhaft empfangsbereit sind.

## 2. Zigbee-Modell

| Element | Wert |
|---|---|
| Rolle | End Device (`ESP_ZB_DEVICE_TYPE_ED`), Rx-off-when-idle (sleepy) |
| Device-ID | Dimmable Light (0x0101) |
| Endpoint | 1 (Profil Home Automation 0x0104) |
| Server-Cluster | Basic (0x0000), Identify (0x0003), Groups (0x0004), Scenes (0x0005), **On/Off (0x0006)**, **Level Control (0x0008)**, **Power Configuration (0x0001)** |
| Optional | Time (0x000A) als Client, falls lokale Zeitfunktionen bleiben |

### Attribut-Mapping auf `led_controller`

| Zigbee | Gerät |
|---|---|
| On/Off `OnOff` = 1 | `led_fade(current, level_to_duty(CurrentLevel), transition)` bzw. `led_on(duty)` |
| On/Off `OnOff` = 0 | Fade auf 0, danach `led_off()` und LEDC stoppen (Energie, siehe power-management.md §3) |
| Level `CurrentLevel` 1–254 | Duty 1–255 (lineares Mapping; Gamma macht `led_controller` intern) |
| Level `TransitionTime` (1/10 s) | Fade-Dauer; kurze Default-Transition (Soft-Start gegen Spannungseinbruch) |
| `OnLevel` / `StartUpOnOff` | Verhalten nach Reset/Batteriewechsel [OFFEN: letzten Zustand aus NVS wiederherstellen?] |

Achtung: `led_controller` hat heute keinen echten One-shot-Fade (FADE läuft endlos hin und her) – siehe
[led-controller.md](led-controller.md). Für Level-Transitions muss ein One-shot-Fade ergänzt werden.

## 3. Sleepy End Device & Energie

Ablauf laut esp-zigbee-sdk-Beispiel „sleepy end device“ (Light Sleep):
- Kconfig: `CONFIG_ZB_ENABLED`, `CONFIG_ZB_ZED`, `CONFIG_PM_ENABLE`, `CONFIG_FREERTOS_USE_TICKLESS_IDLE`,
  `CONFIG_IEEE802154_SLEEP_ENABLE` (Namen gegen SDK-Version prüfen).
- Vor Stack-Start: `esp_zb_sleep_enable(true)`; `esp_pm_configure()` mit `light_sleep_enable = true`.
- Im Signal-Handler `esp_zb_app_signal_handler()` bei `ESP_ZB_COMMON_SIGNAL_CAN_SLEEP` → `esp_zb_sleep_now()`.
- Parent (Router/Koordinator) puffert Nachrichten für das schlafende Gerät; das Gerät holt sie per **Data Poll**.

### Kernkonflikt: Reaktionszeit vs. Akku
- Ein Befehl aus HA wird erst beim nächsten Poll ausgeliefert ⇒ **Latenz ≈ Poll-Intervall**.
- Parents verwerfen gepufferte Nachrichten nach ca. 7,68 s (Indirect-Transmission-Timeout, Standardwert) –
  das Poll-Intervall muss darunter liegen, sonst gehen Befehle verloren.
- Vorschlag [OFFEN]: Fast-Poll (z. B. 250–500 ms) für einige Sekunden nach jeder Interaktion, sonst
  Long-Poll (z. B. 2–5 s). Jeder Poll = kurzer Wakeup mit Radio-RX → dominiert Ruhestrom bei LED aus. Messen!
- End Device Timeout (Keep-alive) mit dem Parent lang konfigurieren, um Rejoins zu vermeiden.

### Weitere Energieaspekte
- TX-Leistung (`esp_zb_set_tx_power`) nur so hoch wie nötig.
- Attribut-Reporting (Akku) selten (Minuten bis Stunden), nicht bei jeder Messung.
- Kein Logging über UART im Release (hält System wach).

## 4. Home Assistant

- **ZHA**: Gerät erscheint als Licht mit Helligkeit, Batterie als Sensor (über Power Configuration).
- **Zigbee2MQTT**: Ohne Converter wird das Gerät ggf. nur generisch erkannt; eigener External Converter möglich [OFFEN].
- Basic-Cluster `ManufacturerName` / `ModelIdentifier` sinnvoll setzen (Erkennung in HA).
- Zeitpläne (früher DS3231-Timer) werden als **HA-Automationen** abgebildet.

## 5. Projektänderungen für Zigbee

- Abhängigkeiten in `main/idf_component.yml`: `espressif/esp-zigbee-lib`, `espressif/esp-zboss-lib` (Versionen passend zu IDF 5.5.x).
- **Partitionstabelle**: aktuell `CONFIG_PARTITION_TABLE_SINGLE_APP=y` – Zigbee benötigt eigene Partitionen
  (`nvs`, `zb_storage`, `zb_fct`) ⇒ Custom-`partitions.csv`.
- NVS initialisieren (`nvs_flash_init()`), Zigbee-Task (`esp_zb_stack_main_loop`) als eigener Task in Komponente `zigbee_light`.
- Pairing/Commissioning: Network Steering beim ersten Start; Factory Reset per Taster (z. B. Long-Press 10 s) [OFFEN].
- Thread-Sicherheit: Zigbee-Attribute nur mit `esp_zb_lock_acquire/release` aus anderen Tasks ändern.
