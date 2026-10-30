# agv-tug

Project repository for 23ELC302, Embedded System Design using ARM.

Firmware for a line-following AGV (automated guided vehicle) tug on an
STM32F401RE (Cortex-M4F). It navigates a station map with A* routing, follows
a line with an IR array, confirms stations by RFID, avoids obstacles, and runs
its control loop under FreeRTOS.

## Repository layout

```
agv-tug/
├── firmware/   STM32CubeIDE project (import this folder)
├── docs/       build notes, RTOS design, bench checklist
├── LICENSE
└── README.md
```

### `firmware/`

| Folder          | Contents                                                              |
|-----------------|-----------------------------------------------------------------------|
| `App/`          | Robot logic: navigation, A*, map, PID, Kalman, odometry, RFID stations, obstacle manager, telemetry, RTOS tasks |
| `DeviceDrivers/`| Hardware drivers: motor, encoder, IR array, MPU6050, OLED, RFID, HC-05, obstacle sensor |
| `Platform/`     | Register-level peripheral setup (clock, GPIO, timers, I2C, SPI, UART, ADC, DMA, interrupts) |
| `Tests/`        | `HardwareTests` build: 21 bench tests selected in `Tests/Src/test_main.c` |
| `Src/`, `Inc/`  | `main.c`, syscalls, `FreeRTOSConfig.h`                                |
| `Startup/`      | Startup assembly                                                      |
| `Drivers/CMSIS/`| Bundled CMSIS headers (no CubeMX install needed)                      |
| `Middlewares/`  | FreeRTOS kernel V10.3.1 (Cortex-M4F GCC port, heap_4)                 |

Linker scripts (`STM32F401RETX_FLASH.ld`, `STM32F401RETX_RAM.ld`), `.project`,
`.cproject` and the `.launch` debug configurations live at the `firmware/`
root because STM32CubeIDE expects them there.

## Building

1. In STM32CubeIDE: **File → Import → Existing Projects into Workspace**, and
   select `firmware/`.
2. Set the active configuration to **HardwareTests**.
3. **Project → Clean**, then **Build Project**.
4. First run with the wheels lifted.

Build configurations:

- `HardwareTests`: builds `Tests/` and replaces `Src/main.c` with
  `Tests/Src/test_main.c`. The test is chosen with `ACTIVE_TEST`
  (currently `TEST_RTOS_LINE_MAP`, the full integrated RTOS run).
- `Debug`: the bare-metal application in `Src/main.c` (TIM5 as the 10 ms
  scheduler, no RTOS tasks, no `Tests/`).
- `Release`: stock CubeIDE configuration. Its source entries cover only
  `Src`, `Startup`, `Inc` and `Middlewares`, so it is not a usable build as-is.

The two `.launch` files (`Debug`, `HardwareTests`) are the matching debug configurations.

## Documentation

- [`docs/rtos-integration.md`](docs/rtos-integration.md): RTOS task design, synchronization, Live Expressions, state values
- [`docs/cmsis-build-fix.md`](docs/cmsis-build-fix.md): why the CMSIS headers are bundled and how the include paths work
- [`docs/bench-checklist.md`](docs/bench-checklist.md): hardware checks to do before trusting autonomous operation
