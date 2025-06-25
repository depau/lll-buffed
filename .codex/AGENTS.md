# Mellow LLL Plus 3D printer filament buffer firmware source code

This is the firmware source code of the Mellow LLL Plus 3D printer filament buffer.

The buffer is a device that ensures a continuous supply of filament to the 3D printer by unwinding a few centimeters of
filament from the spool into a small spring-loaded buffer. Optical sensors detect the buffer position, and trigger a
motor to unwind or rewind the filament as the extruder extrudes/retracts filament.

The firmware is written in C++ and uses PlatformIO for development.

### Directory Structure

- `lib/`: Contains the source code files.
- `src/`: Contains the entry point of the firmware.
- `variants/`: Contains the board pin definitions and linker scripts for the device's microcontroller.
- `platformio.ini`: The PlatformIO project configuration file.

### Development instructions

When making changes to the firmware you must ensure that the code is formatted correctly by running `clang-format` on the source files. You can do this by running the following command in the root directory of the project:

```bash
clang-format -i lib/*.cpp lib/*.h src/*.cpp src/*.h
```

You must also ensure the code builds correctly with PlatformIO. You can do this by running the following command in the root directory of the project:

```bash
pio run -e fly_buffer_f072c8
```
