# LLL Buffed - Mellow LLL Plus Custom Firmware

This is a custom firmware for the Mellow LLL Plus 3D printer filament buffer.

In addition to the original firmware, this firmware implements the following additional features:

- **Filament hold**: When pressing a button briefly, the buffer will hold the filament in place.
- **Continuous run mode**: When a button is double-pressed (or triple-pressed, configurable), the buffer will
  continuously run the motor until a button is pressed again or the timeout (default 1m 30s, configurable) is reached.
  On timeout, the buffer will hold the filament in place.
- **Serial commands and status**: The buffer can be controlled via serial commands, and it provides status updates
  on the current state of the buffer. This allows implementing useful functionality such as assisted loading/unloading
  of filament.
- **Power save mode**: When controlled via serial commands, a power save mode can be enabled that turns off the motor
  after a few seconds of inactivity. Read more about this later in the document.

### Installation

To install the firmware, you will need to use PlatformIO Core. Follow the
official [PlatformIO installation instructions](https://platformio.org/install/cli).

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/depau/lll-buffed.git
    cd lll-buffed
    ```

2. Unscrew and remove the top cover of the buffer, and connect the device to your computer via USB.
3. While holding the `B` (boot) button on the buffer, press and release the `R` (reset) button to put the device into
   bootloader mode. They are located on the PCB between the push/retract buttons. I usually press the `B` button with
   one side of the finger, then rotate the same finger to press and release the `R` button with the other side of the
   finger.
4. Build and upload the firmware using PlatformIO:

   ```bash
   pio run -e fly_buffer_f072c8 -t upload
   ```

If PlatformIO complains that it cannot find the device, try again putting the device into bootloader mode, it may take a
few tries to get the technique right.

### Serial Commands

Most commands have short aliases for convenience. The commands are:

- `push` (`p`): Push the filament forward (continuous run mode).
- `retract` (`r`): Retract the filament (continuous run mode).
- `hold` (`h`): Hold the filament in place.
- `regular` (`n`): Switch to regular mode (push/retract based on sensor inputs).
- `off` (`o`): Switch to regular mode; if the filament is to be held, the motor is turned off instead. (This will be
  reset next time filament is pushed or retracted)
- `move <+/-distance>` (`m <+/-distance>`): Move the buffer a fixed distance in millimeters. A negative distance means
  retracting the filament. After the move, the buffer will hold the filament in place.
- `query` (`q`): Query the current state of the buffer.

A few commands allow tweaking the buffer settings. **The settings are stored in RAM**, so the host software will have
to reapply them after a power cycle.

- `set_timeout <MS>`: Set the timeout for the continuous run and regular modes in milliseconds. Default is 90000 (1
  minute 30 seconds).
- `set_hold_timeout <MS>`: Set the hold timeout in milliseconds. Default is 10000 (10 seconds).
- `set_hold_timeout_enabled <0|1>`: Enable or disable the hold timeout (power save mode). Default is 0 (disabled).
- `set_multi_press_count <N>`: Set the number of button presses required to enter continuous run mode. Default is 2.
- `set_speed <MM/S>`: Set the speed of the motor in millimeters per second. Default is 30.

When the status or the settings change, the buffer will send a status update over serial. The status updates are the
following:

- `mode=<mode>`: The current mode of the buffer. Possible values are:
    - `regular`: Automatic operation based on the optical sensors.
    - `continuous`: Continuous run mode started by a command or a sequence of button presses.
    - `serial`: Move for a fixed distance issued by the host.
    - `hold`: Filament held in place with the motor enabled, ignoring sensor inputs.
    - `manual`: Motor controlled directly by a held button.
- `status=<status>`: The current status of the motor. Possible values are:
    - `push`: Filament pushed forward.
    - `retract`: Filament retracted.
    - `hold`: Motor enabled with zero velocity, either in hold mode or when the buffer is idle in regular mode.
    - `off`: Driver disabled.
- `filament_present=<0|1>`: Whether the filament is present in the buffer (i.e., the sensors detect it).
- `timed_out=<0|1>`: Whether the buffer has timed out in continuous run or regular mode.
- `timeout=<MS>`: The current configured timeout in milliseconds.
- `hold_timeout=<MS>`: The current configured hold timeout in milliseconds (power save mode).
- `hold_timeout_enabled=<0|1>`: Whether the hold timeout (power save mode) is enabled.
- `multi_press_count=<N>`: The number of button presses required to enter continuous run mode.
- `speed=<MM/S>`: The current speed of the motor in millimeters per second.

### Timeout

In regular mode, the buffer will keep pushing or retracting the filament until the sensors detect that the buffer is
empty or full. In continuous run mode, the buffer will keep pushing or retracting the filament until a button is
pressed.

If the buffer does not receive any input (user or sensor) for a certain amount of time, it will time out and hold the
filament in place.

This is to prevent the buffer from running indefinitely if the sensors are not detecting the filament correctly or if
the user forgets to stop the buffer.

Timeouts are reset by any user input (button press) or via serial commands (i.e. `regular`, `push`, `retract`).

### Power Save Mode

When the buffer is controlled via serial commands, it can enter a power save mode that turns off the motor after a few
seconds of inactivity.

This is disabled by default since it can be annoying when operating the buffer manually. The idea is that the printer
host software should enable it when its own filament sensor detects the filament, since when the filament is in all the
way there's no need to hold it in place with the motor.

Operating the buffer manually will disable the power save mode, so the filament won't suddenly start moving after
stopping it manually.
