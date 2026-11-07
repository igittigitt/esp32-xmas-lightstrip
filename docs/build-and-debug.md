# Build, Flash & Debug

## Toolchain

- ESP-IDF **v5.5.1** (lokal z. B. `C:/Users/<user>/esp/v5.5.1/esp-idf`, im Devcontainer `/opt/esp/idf`).
- Target `esp32h2`. Registry-Abhängigkeiten werden beim ersten Build nach `managed_components/` geladen
  (gepinnt in `dependencies.lock`).
- `.clangd` ist für RISC-V-Cross-Compile angepasst (`--target=riscv32-esp-elf`).

## Befehle

```bash
idf.py build
idf.py -p COM5 flash monitor        # Port anpassen; Monitor beenden: Ctrl+]
idf.py menuconfig                   # Änderungen anschließend in sdkconfig.defaults übernehmen!
idf.py size / size-components       # Speicherverbrauch
```

`sdkconfig` ist generiert und gitignored. Persistente Einstellungen gehören in **`sdkconfig.defaults`**.
Nach Änderung von `sdkconfig.defaults` ggf. `sdkconfig` löschen und neu bauen.

## Debug-Hilfen im Code

| Mittel | Wo | Hinweis |
|---|---|---|
| `#define DEBUG 1` | main.c | aktiviert Sleep-Indikator, `debug_task`, EEPROM-Dump, Wakeup-Logs |
| `LOG_LEVEL` | main.c | `ESP_LOG_DEBUG`; für Release/Strommessung reduzieren |
| GPIO4 Sleep-Indikator | `sleep_indicator_*` | HIGH = schläft |
| Konsolenmodus | Taster beim Boot | Timer/Uhr per UART konfigurieren |
| `CONFIG_ESP_SYSTEM_USE_FRAME_POINTER=y` | sdkconfig.defaults | bessere Backtraces |
| `.vscode/launch.json` | | JTAG-Debug über ESP-IDF-Extension |

## Strommessung

Für Aussagen zur Energieeffizienz immer messen (Power Profiler / Shunt + Oszilloskop), mit `DEBUG 0`,
reduziertem Logging und ohne angeschlossenen USB-UART (Rückspeisung verfälscht Messung).
