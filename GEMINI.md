# Mellow LLL Plus 3D printer filament buffer firmware source code

This is the firmware source code of the Mellow LLL Plus 3D printer filament buffer.

The buffer is a device that ensures a continuous supply of filament to the 3D printer by unwinding a few centimeters of
filament from the spool into a small spring-loaded buffer. Optical sensors detect the buffer position, and trigger a
motor to unwind or rewind the filament as the extruder extrudes/retracts filament.

The firmware is written in C++ and uses PlatformIO for development.

### Directory Structure

- `README.md`: Instructions for installing and using the firmware.
- `lib/`: Contains the source code files.
- `src/`: Contains the entry point of the firmware.
- `variants/`: Contains the board pin definitions and linker scripts for the device's microcontroller.
- `platformio.ini`: The PlatformIO project configuration file.

### Development instructions

When making changes to the firmware you must ensure that the code is formatted correctly by running `clang-format` on
the source files. You can do this by running the following command in the root directory of the project:

```bash
clang-format -i lib/*.cpp lib/*.h src/*.cpp src/*.h
```

You must also ensure the code builds correctly with PlatformIO. You can do this by running the following command in the
root directory of the project:

```bash
pio run -e fly_buffer_f072c8
```

You must ensure that unit tests run and pass. You can do this by running the following command:

```bash
pio test -e native -vvv
```

Adding `-vvv` gets you more verbose output, which can be useful for debugging.

You should ensure the unit tests run without any memory leaks. You can build the tests without running them by adding
the `--without-testing` flag (no need to rebuild if you just ran the tests before):

```bash
pio test -e native --without-testing
```

Then you can use `valgrind` to check for memory leaks:

```bash
valgrind --leak-check=full --show-leak-kinds=all .pio/build/native/program
```

`gdb` is installed should you need to debug the tests, but be careful when running it to make sure to quit the gdb
session or else your shell session will be left in a broken state.

Finally, you should run `clang-tidy` to check for any code quality issues and fix any warnings unless it really doesn't
make sense. You can do this by running the following command in the root directory of the project:

```bash
pio check
```

If you decide to ignore a warning, please add a comment in your response explaining why you chose to ignore it.
