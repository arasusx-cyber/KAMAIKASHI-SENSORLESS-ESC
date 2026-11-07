# Firmware setup and UART reference

[Project](../README.md) · [Hardware](HARDWARE.md) · [Current source](../main/CUSTOM_FIRMWARE_ESC.ino)

## Choose a sketch

Each `.ino` is a separate experiment. Build one sketch at a time.

| Sketch | Purpose | UART baud |
| --- | --- | --- |
| [main/CUSTOM_FIRMWARE_ESC.ino](../main/CUSTOM_FIRMWARE_ESC.ino) | Current open-loop sine drive, ADC telemetry and ZC monitoring | 38400 |
| [experimental/6pwm_bridge_test.ino](../experimental/6pwm_bridge_test.ino) | Complementary PWM and per-phase GPIO OFF testing | 9600 |
| [experimental/open_loop_sinusoidal_test.ino](../experimental/open_loop_sinusoidal_test.ino) | Earlier sinusoidal-drive experiment | 38400 |
| [experimental/alt_test_firmware.ino](../experimental/alt_test_firmware.ino) | Alternative bridge/startup experiments | 38400 |

The command table below applies to the current **main** sketch. Experimental files have their own help and behavior.

## Build setup

Use an Arduino STM32 core with an STM32F103C8T6 target and the STM32F1 headers used by the selected sketch. The main sketch assumes a **72 MHz TIM1 clock** when calculating its PWM period. A board/core version is not pinned and no PlatformIO configuration is included.

For Arduino IDE, copy `main/CUSTOM_FIRMWARE_ESC.ino` into a sketch folder named `CUSTOM_FIRMWARE_ESC` and open that file. Record the chosen board variant, clock and core version with your bench results.

Check the pin map and driver polarity against the actual PCB before flashing. Use a current-limited bench supply and verify the idle bridge state before a short motor-spin test. Keep a physical supply disconnect available.

## Terminal

Use **38400 baud** with MCU **PB6 TX**, **PB7 RX** and a common GND. Connect TX to the adapter's RX and RX to its TX using matching 3.3 V logic levels.

On startup the sketch enters freewheel, calibrates ADC offsets, and prints help/status. Commands are single characters.

| Key | Main-sketch action |
| --- | --- |
| Space | Software emergency OFF; force freewheel |
| `f` | Freewheel; clear frequency and modulation targets |
| `e` | Enable RUN |
| `0` | Re-enable RUN with zero targets and neutral PWM |
| `r` | Toggle direction |
| `[` / `]` | Decrease / increase target modulation by 0.01 |
| `-` / `+` (`=`) | Decrease / increase electrical frequency by 0.5 Hz |
| `,` / `.` | Decrease / increase phase offset by 2° |
| `{` / `}` | Decrease / increase raw dead-time setting |
| `z` / `x` | Decrease / increase ZC ADC threshold |
| `t` | Toggle telemetry |
| `1` / `2` / `5` | Telemetry period: 100 / 200 / 500 ms |
| `v` | Toggle verbose ADC telemetry |
| `k` | Toggle ZC telemetry |
| `c` | Recalibrate ADC offsets |
| `p` | Print full status, ADC and ZC readings |
| `h` | Print help |

## Main-sketch configuration

| Setting | Current source value |
| --- | --- |
| PWM / control rate | 20 kHz / 2 kHz |
| Initial target frequency | 5 Hz electrical |
| Initial target modulation | 0.08 |
| Frequency slew | 20 Hz/s |
| Modulation slew | 0.35/s |
| VBAT divider factor | 30.1 |
| Initial raw dead-time setting | 36 |
| Initial ZC threshold | 12 ADC counts |

Verify calibration and dead time on the actual board. Zero-cross detection currently feeds monitoring only; it does not schedule commutation or switch the main sketch into closed-loop operation. The software OFF command also depends on the MCU and UART continuing to run.
