# AGV RTOS bench checks

Please verify these items on the AGV before relying on autonomous operation:

- Confirm the obstacle sensor is wired to PC0 and determine whether its asserted output is low or high. `Obstacle_ReadRaw()` reads PC0; the current debounce logic assumes active low.
- Confirm positive motor commands move both wheels forward. Check left and right independently, then verify turning directions.
- Check that both encoder channels produce counts while the wheels turn, and confirm the reported left/right behavior matches the physical wheels.

The project uses Serial Wire debug on PA13/PA14. PB3 is no longer reserved for SWO.
