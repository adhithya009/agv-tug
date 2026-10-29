# CMSIS build fix

The previous package had stale absolute CMSIS include paths pointing to `C:\Users\HP\...`.
This project now carries the required STM32F4 CMSIS headers inside `Drivers/CMSIS` and uses project-relative include paths:

- `${ProjDirPath}/Drivers/CMSIS/Include`
- `${ProjDirPath}/Drivers/CMSIS/Device/ST/STM32F4xx/Include`

No CubeMX or external STM32Cube firmware path is required for these headers.

After importing/opening the project in STM32CubeIDE:
1. Set active build configuration to `HardwareTests`.
2. Project -> Clean.
3. Build Project.
