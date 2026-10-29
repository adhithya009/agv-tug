# AGV Tug - Full RTOS HardwareTests Integration

This project was built from `AGV_Tug_Firmware_snow.zip` as the source of truth.
The working navigation, map, motor, IR, RFID, obstacle, MPU and OLED drivers were not replaced by the older RTOS project's robot logic.

## Active HardwareTests test

`Tests/Src/test_main.c` now selects:

```c
#define TEST_RTOS_LINE_MAP 21
#define ACTIVE_TEST TEST_RTOS_LINE_MAP
```

Use the **HardwareTests** build configuration.

## No CubeMX dependency

The project uses the existing register-level files:

- `Platform/Src/system_clock.c`
- `Platform/Src/gpio_cfg.c`
- `Platform/Src/timer_cfg.c`
- `Platform/Src/i2c_cfg.c`
- `Platform/Src/spi_cfg.c`
- `Platform/Src/uart_cfg.c`

`FreeRTOSConfig.h` is hardcoded in `Inc/FreeRTOSConfig.h`.
The FreeRTOS kernel is under `Middlewares/Third_Party/FreeRTOS`.

## Timer change

For test 21, `test_main.c` calls `TIMER_InitForRTOS()`.
It starts the robot hardware timers but does not start TIM5 as the 10 ms scheduler.
FreeRTOS SysTick + `vTaskDelayUntil()` now schedule the 100 Hz control loop.

All older bare-metal HardwareTests still use `TIMER_Init()` and TIM5 if you change `ACTIVE_TEST` back to one of them.

## RTOS tasks

### CONTROL - highest application priority, 100 Hz
Owns all motion decisions:

- 2 second stationary gyro calibration
- start RFID gate transition
- line IR sample
- MPU sample
- `Navigation_UpdateWithSensors()`
- obstacle-triggered A* replan and `Navigation_StartMidEdgeUTurn()`
- path-index validation
- route timeout
- route-complete destination wait
- all Navigation/Motor state changes

This task is the only task allowed to decide navigation/motor behavior.

### OBSTACLE - 100 Hz

- calls `Obstacle_Update()`
- reads the debounced obstacle result
- publishes an event bit
- never commands motors and never changes the A* path

### RFID - 10 Hz

Preserves the behavior of the uploaded integrated test:

- after calibration, confirms the expected start RFID before route motion
- while running, polls for the destination RFID
- after route-complete, continues destination confirmation polling
- publishes event bits; it does not command motors

Current uploaded route configuration is preserved:

- start: `MAP_S1`
- destination: `MAP_S4`
- heading: `MAP_DIR_SOUTH`

### DISPLAY - state-change driven

- owns OLED updates
- displays calibration/start-RFID/running/destination/error states
- uses the same I2C mutex as the MPU so OLED and MPU cannot transact on I2C1 simultaneously

## Synchronization

- FreeRTOS event group: start RFID, destination RFID, obstacle detected
- FreeRTOS mutex: I2C1 shared by MPU6050 and SSD1306 OLED
- ControlTask is the sole navigation/motion owner

## Encoders

The uploaded `test_main.c` explicitly says encoders are removed/bypassed and the integrated line-map test keeps encoder debug values zeroed. Therefore this RTOS integration does **not** call `Encoder_Update()` from the control loop. It does not reintroduce encoder control that is absent from the uploaded working test.

## Important obstacle assumption

The existing obstacle driver is unchanged:

- sensor: A5 / PC0
- active level: LOW
- `OBSTACLE_DEBOUNCE_COUNT` is currently 10 samples (100 ms at 100 Hz)

If physical polarity is opposite, change `OBSTACLE_ACTIVE_LEVEL` in `DeviceDrivers/Src/obstacle.c`.

## Build steps

1. Import project into STM32CubeIDE.
2. Select `HardwareTests` configuration.
3. Project -> Clean.
4. Build Project.
5. Confirm zero errors before flashing.
6. First run with wheels lifted.

Useful Live Expressions:

- `debug_rtos_test_stage`
- `debug_rtos_state`
- `debug_rtos_route_found`
- `debug_rtos_ir_pattern`
- `debug_rtos_line_error`
- `debug_rtos_path_index`
- `debug_rtos_nav_mode`
- `debug_rtos_obstacle_raw`
- `debug_rtos_obstacle_detected`
- `debug_rtos_replan_count`
- `debug_rtos_rfid_station`
- `debug_rtos_start_rfid_ok`
- `debug_rtos_dest_rfid_ok`
- `debug_rtos_mpu_calibrated`
- `debug_rtos_oled_ok`
- `debug_rtos_control_stack_free`

RTOS state values:

- 0 = calibrating
- 1 = waiting for start RFID
- 2 = running
- 3 = waiting for destination RFID
- 4 = destination
- 5 = A* failure
- 6 = path index error
- 7 = timeout
- 8 = obstacle blocked / no alternate path
