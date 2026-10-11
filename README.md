# tempLoggerNG

A four-channel PT-100 temperature logging device built around a Texas
Instruments ADS1248 precision delta-sigma ADC and an ATmega328p
microcontroller with V-USB (software-only USB, no dedicated USB
controller chip). The device connects directly to a PC over USB and is
controlled by a small command-line tool that triggers measurements and
logs resistance/temperature readings to a file.

https://emsyfs.blogspot.com/2017/07/pt-100-temperature-logger.html

## Features

- 4 independent PT-100 RTD input channels, read via a single ADS1248
  24-bit ADC using its internal multiplexer and a programmable current
  source (IDAC) to excite each sensor.
- Full 4-wire measurement on all 4 channels, eliminating lead
  resistance errors for accurate readings even with long sensor
  cables.
- On-board diode temperature sensor, used to compensate for
  temperature drift of the measurement electronics themselves.
- Linear temperature-compensation coefficients (`a`, `b`) stored in the
  ATmega's EEPROM, so each device can be individually calibrated and
  keeps its calibration across power cycles.
- USB connectivity without a dedicated USB interface chip, using the
  V-USB software USB implementation on the ATmega328p.
- A command-line host tool (`templogger`) that:
  - reads out the 4 channels at a configurable sample rate,
  - converts raw resistance to temperature using the Callendar-Van
    Dusen equation for PT-100 sensors,
  - logs readings (time, diode voltage, and temperature or resistance
    per channel) to a plain-text data file,
  - optionally applies/updates the temperature-compensation
    calibration,
  - can read/write individual ADS1248 configuration registers for
    debugging, and
  - includes a USB round-trip echo self-test.
- Full KiCad hardware design (schematic, PCB layout, custom
  footprints, Specctra autorouter export, and a rendered schematic
  PDF) for building the device yourself.

## Repository layout

```
firmware/     ATmega328p firmware (C), including the V-USB driver and
              the ADS1248 driver
commandline/  Host-side command-line tool ("templogger") using libusb
hardware/     KiCad schematic/PCB project, footprints, and a PDF of
              the schematic
```

## Hardware

The design files live in `hardware/` as a KiCad project
(`tempLoggerNG.pro`, `tempLoggerNG.sch`, `tempLoggerNG.kicad_pcb`). A
rendered PDF of the schematic is included as
`tempLoggerNG-schematic.pdf` if you just want to look at the circuit
without opening KiCad.

> **Note:** This project was originally designed in an older version
> of KiCad (legacy schematic/library format). Opening it in a modern
> KiCad release will likely trigger the automatic "symbol rescue"
> feature, which copies any symbols it can't resolve against current
> libraries into local rescue files. This is expected and does not
> modify the original schematic; if KiCad has already run through
> this once in your checkout, you may need to clean up before
> committing anything.

## Firmware

The firmware lives in `firmware/` and is built with `avr-gcc` for the
ATmega328p.

### Requirements

- `avr-gcc` / `avr-libc` (AVR toolchain)
- `avrdude`
- An AVR ISP programmer, such as a USBasp (the Makefile defaults to
  `avrdude -c usbasp -p m328p` — edit `firmware/Makefile` if you use a
  different programmer)

### Build and flash

```sh
cd firmware
make          # builds main.hex
make flash    # flashes main.hex to the ATmega328p via avrdude/usbasp
```

The ATmega328p is clocked at 12 MHz (`F_CPU`, set in `main.c` and the
Makefile) and must be fused accordingly (external crystal, no
clock-divide) before flashing application code — the fuse bytes are
not set by this Makefile and must be programmed separately with
`avrdude`, matching whatever oscillator is fitted on your board.

Other useful targets:

```sh
make disasm   # disassemble main.bin with avr-objdump
make clean    # remove build artifacts
```

The firmware never leave debug output enabled in a flashed device —
the comment in the Makefile is explicit that debug builds distort the
timing the USB stack depends on.

## Command-line tool

The host tool lives in `commandline/` and talks to the device over USB
using libusb (the classic 0.1 API, via `libusb-config`).

### Requirements

- `gcc`
- `libusb` (0.1, providing `libusb-config`)

### Build

```sh
cd commandline
make
```

This produces a `templogger` binary. The device identifies itself
over USB using obdev's free shared VID/PID (`0x16C0`/`0x05DC`) with
manufacturer string `www.obdev.at` and product string `Temp-Logger`,
so no special udev rule beyond generic USB access permissions is
normally required — just make sure your user can access the USB
device (e.g. via a udev rule granting access for that VID/PID, or by
running as root).

### Usage

```
usage: ./templogger [ch1] [ch2] [ch3] [ch4] [all] [options]
```

Specify which channel(s) to log with one or more of `ch1`, `ch2`,
`ch3`, `ch4`, or `all`.

Options:

```
 -v          : verbose
 -q          : quiet
 -o filename : name of data file. default is "temperature.dat"
 -N n        : stop measurement after n samples
 -t t        : stop measurement after t seconds
 -d delta    : take a sample every delta seconds. Default is 0.5, min is 0.15
 -r          : read registers (for debugging purposes)
 -h          : output this help
```

Example — log channels 1 and 2 to `run1.dat`, once per second, for 60
samples:

```sh
./templogger ch1 ch2 -o run1.dat -d 1 -N 60
```

### Calibration procedure

Each device's on-board diode-based temperature compensation uses two
coefficients, `a` and `b`, such that:

```
R_comp = a - b * V_diode
R      = R_measured - R_comp
```

To determine and store them for a given device:

1. Connect a fixed, known resistor (e.g. 100 Ω) in place of a sensor.
2. Disable temperature compensation with `-x` and log resistance
   directly with `-R`.
3. Heat up the device itself (not the resistor) and record data while
   it cools down.
4. Plot `V_diode` against `R_measured - 100 Ω` and fit a straight line
   to obtain `a` and `b`.
5. Write the coefficients to the device's EEPROM:
   ```sh
   ./templogger -c <a> <b>
   ```

Once written, the coefficients are read back from EEPROM
automatically and applied during normal logging unless `-x` is given.

### Self-test

```sh
./templogger test
```

Sends 1000 random values to the device and checks that they are
echoed back correctly, as a basic USB communication sanity check.

## License

The firmware and command-line tool (`firmware/`, `commandline/`) are
licensed under the GNU General Public License v2 (see `LICENSE`). The
bundled V-USB driver (`firmware/usbdrv/`) is dual-licensed under GPLv2
or a commercial license from Objective Development — see
`firmware/usbdrv/License.txt` and
`firmware/usbdrv/CommercialLicense.txt` for details, and
`firmware/usbdrv/USB-ID-FAQ.txt` / `USB-IDs-for-free.txt` regarding use
of the shared VID/PID.

The hardware design (`hardware/`) is licensed under the CERN Open
Hardware Licence Version 2 - Strongly Reciprocal (CERN-OHL-S v2), see
`LICENSE-HARDWARE`.
