# AGENTS.md

Tartarugo is a smart cat feeder: an ESP32 (esp-wrover-kit, Arduino framework) that watches BLE beacons worn by cats, opens/closes two servo lids based on RSSI thresholds, and dispenses food with a 4-wire stepper. It exposes a plain-HTTP JSON API (port 80, no auth) for status, manual control, scheduling, and settings.

## Layout

- `src/main.cpp` — the whole app: BLE scanning, lid logic, HTTP API, NTP, schedule, NVS persistence, OTA.
- `src/motor-utils.h` — servo (lids) and stepper (feeding) drivers, header-style with globals (no .cpp).
- `src/time-utils.h` — time helpers, `ScheduleItem` struct, `SCHEDULE_BLOB_VERSION`.
- `src/WiFiCredentials.h` — defines `ssid`/`password`. **Gitignored; must be created locally** or the build fails.
- `include/generated/openapi.h` — **generated at build time**, do not edit (see OpenAPI below).
- `openapi.yaml` — API spec, embedded in firmware and served at `/openapi`.
- `tools/generate_openapi_header.py` — SCons pre-build script (`extra_scripts`) that converts `openapi.yaml` into `include/generated/openapi.h`.
- `redocly.yaml` — redocly lint config for `openapi.yaml`.
- `platformio.ini` — envs: `usb`, `ota-MILO`, `ota-NINA`, `ota-TEST`. Contains LAN IPs; currently gitignored — do not commit.
- `test/`, `lib/` — empty, default PlatformIO placeholders.

## Build, flash, monitor

```bash
pio run                                            # build (auto-generates openapi.h)
pio run -t upload --environment usb                # flash over USB (esptool)
pio run -t upload --environment ota-NINA           # flash over ESP OTA (192.168.1.12)
pio device monitor --environment usb               # serial, 115200 baud
python3 tools/generate_openapi_header.py           # regenerate the header standalone
npx @redocly/cli lint openapi.yaml                 # lint the API spec
```

There is no test suite. Verify changes by building, flashing, and exercising the device over serial monitor and the HTTP API.

## Hardware pins (motor-utils.h, main.cpp)

| Pin | Function |
|-----|----------|
| 15 | Lid 1 servo (main lid) |
| 4  | Lid 2 servo (feeding chute) **and** status flash LED (`FLASH_PIN`) — shared |
| 13, 12, 14, 27 | Stepper, FULL4WIRE (wired 13→1, 14→2, 12→3, 27→4) |

Servo angle 0 = open, 180 = closed; servos are attached only while moving / recently active and detached ~10s after closing (see `checkLid`).

## How it works

- **Cat detection**: continuous NimBLE scan (main.cpp `scanCallbacks`). Each configured cat has a MAC; RSSI is smoothed over an 8-sample ring buffer (`averageRSSI`, guarded by `rssiMux`). Cats not seen for 3s decay toward -99 (`checkLastSeen`).
- **Auto lid logic** (`checkLid`, runs every 500ms via `everyPeriod`): a cat with `canFeed=true` above `openBeaconThresholdRSSI` (default -60) opens lid 1; below `closeBeaconThresholdRSSI` (default -73) it closes. A non-`canFeed` cat closer than the target also forces closed (anti-snatching). Manual API lid calls set `lidOverride`, disabling auto mode until `/api/lid/auto` is called.
- **Feeding** (`feedAmount`/`updateStepper`): stepper runs a multi-leg pattern `{-250, 0, -250, 0, <amount>}`; only the last leg carries the amount. `updateStepper()` is called every loop.
- **Schedule** (`runSchedule`): time-based entries, either a feeding (opens lid 2, feeds, saves) or a lid 2 open/close. Matches on `tm_sec == 0`, so it depends on `loop()` hitting that exact second.
- **Health**: `healthCounter` increments every 500ms tick and is reset by a `/api/status/prometheus` scrape; at 100 it reboots (`checkHealth` — currently commented out in `everyPeriod`).
- **Time**: NTP via `pool.ntp.org`, hardcoded GMT+1 / DST+1 (`gmtOffset_sec`, `daylightOffset_sec`).

## HTTP API

Endpoints (see `openapi.yaml` for full details): `GET /api/status`, `GET /api/status/prometheus` (Prometheus text, but sent with `Content-Type: application/json`), `GET /api/logs`, `POST /api/feed`, `POST /api/schedule`, `GET /api/schedule`, `POST /api/settings`, `GET /api/settings`, `POST /api/lid/open|close|auto` (optional `lid=lid2` param), `POST /api/flash/on|off`, `GET /openapi`.

**When adding/changing an endpoint: update `openapi.yaml` in the same change.** It is compiled into the firmware and served at `/openapi`, so the spec is the contract. Run the redocly lint afterwards.

## Persistence (ESP NVS via `Preferences`)

- Namespace `catfeeder`: last feed time/amount, schedule blob, cat list, RSSI thresholds. All NVS access is wrapped in `xSemaphoreTake/Give(nvsMutex)`.
- Namespace `logs`: circular buffer of 50 events (`MAX_LOGS`), written by `logEvent` (reboot reason, startup time, ...), read by `GET /api/logs`.
- **Schedule blobs are versioned**: if you change the `ScheduleItem` layout, bump `SCHEDULE_BLOB_VERSION` in `time-utils.h` so old blobs are discarded on load.

## Conventions

- Arduino-embedded C++: header files with globals (no class per motor, no .cpp in `src/`), brace-on-next-line style, 4-space indent.
- Memory is tight (ESP32 + ArduinoJson dynamic docs + BLE + web server); prefer static buffers over `String` concatenation in hot paths.
- `main.cpp` is the single translation unit that includes everything — watch for symbol collisions when adding headers.
- Keep changes to the device behavior (thresholds, angles, movement patterns) as constants at the top of the relevant file.

## Gotchas

- `FLASH_PIN` (4) and `lid2ServoPIN` (4) are the same GPIO — driving the flash while the lid 2 servo is attached interacts; be careful changing either.
- `openLid`/`closeLid` take a bitmask (`Lid` enum), but the API maps `lid=lid2` to `LID_2` and default to `LID_1`; note `/api/lid/close` does NOT set `lidOverride` for lid 2.
- The OpenAPI header embeds the spec verbatim; the YAML must stay ASCII-safe for `tools/generate_openapi_header.py` escaping.
- OTA envs hardcode IPs in `platformio.ini`; `usb` is the safe default for local dev.
