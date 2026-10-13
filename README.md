# Arduino-Inverted-Pendulum

Balances an inverted pendulum on a cart that moves along a linear track. Two PID controllers are chained together (cascade control): one keeps the pendulum upright, and the other keeps the cart near the middle of the track.

Derived from krz4tw's [Inverted-Pendulum-PID-control](https://github.com/krz4tw/Inverted-Pendulum-PID-control/blob/master/Cascading_PID_s_lin_kd_denoised.ino).

## Hardware

1. An Adafruit Motor Shield driving one DC motor that moves the cart (`AFMotor`).
2. Two quadrature encoders, one for the cart's position and one for the pendulum's angle (`Encoder`). A full pendulum turn is 1024 counts.
3. Two end-stop switches, on pins 9 and 10.
4. A potentiometer on A5 that adds a small trim to the "vertical" angle.

## How it works

### Setup (runs once)

1. Sets both encoders to 0, so the pendulum has to be held straight up and the cart centered at power-on.
2. Starts both PID controllers with outputs limited to ±255 and a 2 ms sample time.

### Control loop (runs repeatedly)

1. Reads the potentiometer to get the angle trim.
2. If either end-stop is pressed, it cuts the motor and waits there until the switch is released.
3. Reads the cart position and pendulum angle.
4. **Outer loop (cart position PID):** compares the cart position to the center (`lin_Setpoint = 0`). Its output becomes the target angle for the pendulum (`rot_Setpoint`). In effect it tilts the target slightly so the cart drifts back toward center.
5. **Inner loop (angle PID):** compares the pendulum angle to that target and produces a motor command (`Output`).
6. Converts the command into a motor speed between 60 and 255, starting at 60 so the motor isn't stuck in its dead zone, and drives the cart left or right.
7. **Safety cutoffs:** stops the motor if the cart is outside ±13000 counts or the pendulum tilts more than 400 counts, where it can't be caught again.
8. Prints cart position and setpoint over serial for the Arduino Serial Plotter. The fixed ±5000 values keep the plot's axis from rescaling.

## Code that's present but does nothing to the control

1. Cart speed and pendulum speed are calculated as averages over the last 400 cycles, but the PIDs never use them.
2. The five 8000-element data-logging arrays are declared but never written to.
3. Three other gain sets (PI only, PD only, and a tuning set) are commented out.
