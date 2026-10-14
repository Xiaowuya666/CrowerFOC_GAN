# CrowerFOC

STM32C562CET6 single-axis motor-drive firmware with a PyQt5 commissioning and diagnostic application.

## Project status

The project is in active bench-validation. USB CDC, parameter exchange, PWM/FOC, encoder access, current/voltage telemetry, calibration, protections, and the desktop application have been exercised. The current-loop sweep is implemented, but recent measurements show a resonance/peaking region and invalid points around 1.25–1.8 kHz; measured −3 dB bandwidth is therefore unresolved. A 1 kHz pass flag is not a substitute for stable, complete frequency-response validation. High-current/continuous thermal capability and real CAN-bus operation are not fully qualified.

Check the current, dated validation notes before operating the motor. Treat software current limits as configured limits, not verified board ratings.

## Quick start

- User setup and safe operation: [Quick use guide](docs/CrowerFOC_快速使用指南.md)
- Developer, flashing, and bench procedures: [Online test and debug manual](docs/CrowerFOC_在线测试与调试手册.md)
- CAN frame layout and custom current semantics: [MIT-layout CAN protocol](docs/CrowerFOC_MIT_CAN协议说明.md)
- Current-loop sweep record: [2026-10-14 sweep](docs/validation/current_loop_sweep_2026-10-14.csv) and [plot](docs/validation/current_loop_sweep_2026-10-14.png)

Launch the desktop UI on Windows with `HostUI\run_gui.bat`. Build firmware from `Software\MotorTest_cmake` using the `debug_GCC_STM32C562CET6` CMake preset. Flash only after building; see the detailed manual for probe-access caveats.

## Layout

- `Software/MotorTest_cmake/`: STM32 firmware, Cube-generated support, CMake build and flash scripts.
- `HostUI/`: PyQt5 GUI, protocol/client modules, and host-side tests.
- `Hardware/`: schematics, netlists, and component references.
- `tools/`: engineering utilities and bench-test scripts. Local captures and generated results are excluded from Git; curated evidence belongs in `docs/validation/`.
- `docs/`: user, developer, protocol, and validation documentation.

## Host tests

```powershell
cd HostUI
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## License and third-party components

Check the license and notices shipped with the STM32Cube-generated sources and bundled middleware before redistributing this project. This repository does not replace or override those component licenses.
