# Inverted Pendulum on a Cart Simulation and Control

This MATLAB project simulates and controls an inverted pendulum on a cart using either a PID (Proportional-Integral-Derivative) or LQR (Linear-Quadratic Regulator) control strategy. The goal is to balance the pendulum in the upright position and control the cart's position.

## Files Included
- `main.m`: The main script that runs the simulation.
- `Euler_2nd_order.m`: Function for Euler integration to update the system state.
- `Visual.m`: Function to visualize the inverted pendulum system.
- `InvertedPendulum.m`: Function to compute the accelerations of the cart and pendulum.

## How to Run
1. Make sure all the required files (`main.m`, `Euler_2nd_order.m`, `Visual.m`, `InvertedPendulum.m`) are in the same directory.
2. Open MATLAB and navigate to the directory containing the files.
3. Run the `main.m` script.
