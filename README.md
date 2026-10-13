# Arduino-Inverted-Pendulum

What it does: it balances an inverted pendulum on a cart that moves along a track. Two PID controllers are chained together: one keeps the pendulum upright, and the other keeps the cart near the middle of the track.

Hardware it expects
An Adafruit Motor Shield driving one DC motor that moves the cart (AFMotor).
Two quadrature encoders, one for the cart’s position and one for the pendulum’s angle (Encoder). The comments say a full pendulum turn is 1024 counts.
Two end-stop switches, on pins 9 and 10.
A potentiometer on A5 that adds a small trim to the “vertical” angle.
Setup (runs once)
Sets both encoders to 0, so the pendulum has to be held straight up and the cart centered at power-on.
Starts both PID controllers with outputs limited to ±255 and a 2 ms sample time.
Control loop (runs repeatedly)
Reads the potentiometer to get the angle trim.
If either end-stop is pressed, it cuts the motor and waits there until the switch is released.
Reads the cart position and pendulum angle.
Outer loop (cart position PID): compares the cart position to the center (lin_Setpoint = 0). Its output becomes the target angle for the pendulum (rot_Setpoint). In effect it tilts the target slightly so the cart drifts back toward center.
Inner loop (angle PID): compares the pendulum angle to that target and produces a motor command (Output).
Converts the command into a motor speed between 60 and 255, starting at 60 so the motor isn’t stuck in its dead zone, and drives the cart left or right.
Safety cutoffs: stops the motor if the cart is outside ±13000 counts or the pendulum tilts more than 400 counts, where it can’t be caught again.
Prints cart position and setpoint over serial for the Arduino Serial Plotter. The fixed ±5000 values keep the plot’s axis from rescaling.
Code that’s present but does nothing to the control
Cart speed and pendulum speed are calculated as averages over the last 400 cycles, but the PIDs never use them.
The five 8000-element data-logging arrays are declared but never written to.
Three other gain sets (PI only, PD only, and a tuning set) are commented out.

The code comes from krz4tw’s “Cascading PID” sketch, which is credited on line 1. The general technique is called cascade control: an outer loop sets the target for an inner loop.
