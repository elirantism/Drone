# Drone

A from-scratch quadcopter flight controller for the Teensy 4.x, written as a single
Arduino sketch. It reads a PPM receiver and an MPU-6050 gyro, runs a rate-mode PID
loop at 250 Hz, and drives four ESCs over 250 Hz PWM.

## Hardware

| Part              | Notes                                                                                                                         |
| ----------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| Flight controller | Teensy 4.0 / 4.1 (uses `analogWriteFrequency`, `analogWriteResolution`, and the `PulsePosition` library, all Teensy-specific) |
| IMU               | MPU-6050 on I2C address `0x68`, 400 kHz bus                                                                                   |
| Receiver          | Any RC receiver with a PPM (CPPM) sum-signal output                                                                           |
| ESCs              | 4x PWM ESCs, 1000-2000 us throttle range                                                                                      |

### Pinout

| Pin     | Function                                |
| ------- | --------------------------------------- |
| 1       | Motor 1 (front right)                   |
| 2       | Motor 2 (rear right)                    |
| 3       | Motor 3 (rear left)                     |
| 4       | Motor 4 (front left)                    |
| 13      | Built-in LED, lit once arming completes |
| 14      | PPM input from the receiver             |
| 18 / 19 | I2C SDA / SCL to the MPU-6050           |

Motors are mixed for an X-frame layout: the motor positions above follow from the
signs in `applyMotorOutputs()`. Diagonal pairs counter-rotate, so 1 and 3 spin one
direction and 2 and 4 the other - confirm against your own frame and ESC order before
the first spin-up.

### Receiver channels

The sketch reads the PPM stream into `receiverValue[]` in channel order:

| Index | Channel  |
| ----- | -------- |
| 0     | Roll     |
| 1     | Pitch    |
| 2     | Throttle |
| 3     | Yaw      |

## Build and flash

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and
   [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html). Teensyduino ships both
   `Wire` and `PulsePosition`, so there is nothing else to install.
2. Open `drone.ino`, select your Teensy board under **Tools > Board**.
3. Upload. The serial monitor runs at 57600 baud.

## How it works

`setup()`:

- Wakes the MPU-6050 (clears the sleep bit in `PWR_MGMT_1`) and sets the ESC PWM
  frequency to 250 Hz.
- Averages 2000 gyro samples to measure the resting bias on each axis, and stores it
  as the calibration offset. **Keep the drone perfectly still while this runs.**
- Runs an arming wait that holds while the throttle channel reads between 1020 and
  1050 us, then lights the onboard LED on pin 13. Note that `receiverValue[2]` is
  still 0 at that point, so as written this loop falls through immediately and the
  board arms without waiting for the stick - see [Safety](#safety).

`loop()` runs on a fixed 4 ms (250 Hz) schedule, busy-waiting on `micros()` at the end
of each iteration so the PID timing stays constant:

1. Read the receiver and convert stick deflection into desired angular rates
   (`0.15 deg/s` per microsecond of stick travel, so full stick is roughly 75 deg/s).
2. Read the gyro and subtract the startup calibration to get the measured rate.
3. Run `pidControlLoop()` once per axis. The integral term uses trapezoidal
   integration and both it and the total output are clamped to +/-400.
4. Mix the three corrections into four motor outputs, clamp each to 1000-2000 us, and
   write them out at 12-bit resolution.

Below 1050 us of throttle the mixer cuts all four motors to idle and zeroes the PID
integrators and previous-error terms, so the aircraft cannot spool up from wind-up
after a low-throttle moment.

## Tuning

PID gains live at the top of `drone.ino`:

```cpp
float pConstantRollPitch = 0.6f;
float iConstantRollPitch = 3.5f;
float dConstantRollPitch = 0.03f;

float pConstantYaw = 2.0f;
float iConstantYaw = 12.0f;
float dConstantYaw = 0.0f;
```

These are a starting point for one particular frame. Retune them for yours: props off,
craft restrained, one axis at a time.
