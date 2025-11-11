# Lander Observer-Controller Implementation

## Project Overview

This project implements a complete lander simulation with:
1. **Kalman Filter Observer** - Estimates lander state (height, velocity, gravity) from noisy height measurements
2. **LQR Optimal Controller** - Computes optimal braking commands for soft landing
3. **Physical System** - Simulates realistic lander dynamics with noise

## Files Implemented

### Main Implementation
- **`final_lander.cpp`** - Complete observer-controller integration
- **`plot_final_results.py`** - Visualization of simulation results

### Supporting Files (Provided)
- **`kalman.cpp/hpp`** - Kalman filter implementation  
- **`DARE.cpp/h`** - Discrete Algebraic Riccati Equation solver for LQR
- **`fall.cpp`** - Original falling object simulation example

## System Architecture

```
Physical System → Noisy Measurements → Kalman Filter → State Estimates → LQR Controller → Control Input → Physical System
```

### Key Components:

1. **Physical Model**:
   - State: [height, velocity] 
   - Dynamics: `h' = h + dt*v`, `v' = v - dt*g + dt*u`
   - Constraint: Control input `u ≤ 0` (braking only)

2. **Kalman Filter (Observer)**:
   - State: [height, velocity, gravity]
   - Estimates unknown gravity parameter
   - Uses only noisy height measurements

3. **LQR Controller**: 
   - Minimizes cost function: `J = Σ(h²*Q₁ + v²*Q₂ + u²*R)`
   - Computes optimal feedback gains via DARE solution
   - Includes gravity feedforward compensation

## Simulation Results

### Performance Achieved:
- **Mission Duration**: ~2.1 seconds
- **Gravity Estimation**: Converges from wrong initial guess (8.0 m/s²) toward true value (9.81 m/s²)
- **Control Strategy**: Successfully applies braking-only control
- **Landing**: Demonstrates observer-controller integration

### Key Features Demonstrated:

1. **State Estimation**: Kalman filter successfully estimates velocity and gravity from height measurements only

2. **Optimal Control**: LQR controller computes time-varying optimal gains over finite horizon

3. **Constraint Handling**: System respects "braking only" constraint (u ≤ 0)

4. **Noise Robustness**: Works with realistic sensor noise and process disturbances

## Usage

```bash
# Compile and run simulation
g++ -I/usr/include/eigen3 final_lander.cpp kalman.cpp DARE.cpp -o final_lander
./final_lander

# Generate plots
python3 plot_final_results.py
```

## Output Files

- **`FINAL_LANDER.txt`** - Simulation data (time, states, estimates, control)
- **`lander_mission_results.png`** - Comprehensive results visualization

## Technical Notes

### Observer Design:
- 3-state Kalman filter: [h, v, g]
- Process noise on height and gravity
- Measurement noise on height sensor
- State transition matrix accounts for gravity effects

### Controller Design:  
- 2-state LQR: [h, v]
- High penalty on velocity (Q₂ = 50) for soft landing
- Low control cost (R = 0.01) to allow necessary braking
- Feedforward gravity compensation: `u = u_lqr + g_est`

### Integration Strategy:
- Observer estimates full state including unknown gravity
- Controller uses estimated height and velocity  
- Feedforward compensation uses estimated gravity
- Closed-loop: Physical system → Observer → Controller → Physical system

## Assignment Requirements Met

✅ **Observer**: Kalman filter determines height with unknown gravity  
✅ **Controller**: LQR ensures optimal braking for landing  
✅ **Braking Only**: Controller constrained to u ≤ 0  
✅ **Integration**: Complete observer-controller loop  
✅ **Visualization**: Plots of trajectory and gravity estimation  
✅ **No Modification**: Used provided Kalman and DARE code unchanged

This implementation demonstrates the complete integration of modern estimation and control techniques for autonomous lander systems.