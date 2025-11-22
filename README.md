
# State Estimation and LQR Control for Lunar Lander

This project implements a complete state estimation and optimal control system for a lunar lander simulation using Kalman filtering and Linear Quadratic Regulator (LQR) control.

## Overview

The system simulates a falling object (lunar lander) under the influence of gravity with process and measurement noise. A Kalman filter estimates the system state (position, velocity, and gravity), while an LQR controller provides optimal control inputs to regulate the descent.

## Key Features

- **Kalman Filter**: Estimates position, velocity, and gravity from noisy position measurements
- **LQR Control**: Computes optimal control inputs using Discrete Algebraic Riccati Equation (DARE)
- **Delayed Control**: LQR control starts after 40 steps to allow proper state estimation
- **Noise Modeling**: Realistic process and measurement noise simulation
- **Real-time Visualization**: Gnuplot integration for live plotting of results

## System Architecture

### State Vector
The system tracks a 3-dimensional state:
- x₁: Position (height)
- x₂: Velocity  
- x₃: Gravity (treated as unknown constant to be estimated)

### Dynamics Model
```
height = height + dt*velocity + noise
velocity = velocity - dt*gravity + dt*control_input + noise
gravity = gravity (constant)
```

## Files

- `main.cpp`: Main simulation loop integrating Kalman filter and LQR controller
- `kalman.h/cpp`: Kalman filter implementation for state estimation
- `DARE.h/cpp`: Discrete Algebraic Riccati Equation solver for LQR gains
- `fall.cpp`: Physics simulation of falling object dynamics

## Building and Running

```bash
g++ -I /usr/include/eigen3 -I . main.cpp DARE.cpp kalman.cpp -o lander
./lander
```

**Requirements**: Eigen3 library, gnuplot for visualization

## Results

The simulation outputs:
- Real-time plot showing measured vs estimated vs true trajectories
- Data file with time series of all state variables and control inputs
- Comparison of system behavior before and after control activation

This project demonstrates practical implementation of modern control theory concepts including optimal state estimation, optimal control, and their integration in a realistic simulation environment.
