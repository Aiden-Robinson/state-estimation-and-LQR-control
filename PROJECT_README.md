# Lander Simulation with Observer and Optimal Controller

## Project Overview
Implementation of a lunar lander simulation featuring:
- **Kalman Filter Observer**: Estimates height, velocity, and unknown gravity from noisy measurements
- **LQR Optimal Controller**: Computes optimal braking commands for soft landing
- **Physical System**: Realistic lander dynamics with noise and control constraints

## Key Files

### Core Implementation
- `final_lander.cpp` - Main simulation integrating observer and controller
- `DARE.cpp`, `DARE.h` - Discrete Algebraic Riccati Equation solver (LQR controller)
- `kalman.cpp`, `kalman.hpp` - Kalman filter implementation (observer)

### Utilities
- `plot_final_results.py` - Visualization script for simulation results
- `fall.cpp` - Original physics reference implementation
- `main.cpp` - Original DARE controller demonstration

### Results
- `FINAL_LANDER.txt` - Simulation data output
- `lander_mission_results.png` - Trajectory and estimation plots
- `IMPLEMENTATION_SUMMARY.md` - Detailed technical documentation

## How to Run

### Compile and Run Simulation
```bash
g++ -I/usr/include/eigen3 final_lander.cpp kalman.cpp DARE.cpp -o final_lander
./final_lander
```

### Generate Plots
```bash
python3 plot_final_results.py
```

## Mission Objectives
✅ Observer determines height with unknown gravity  
✅ Controller ensures velocity = 0 at height = 0  
✅ Only braking allowed (no upward thrust)  
✅ Successful integration without modifying provided Kalman/DARE code  
✅ Complete visualization and analysis  

## Results
The simulation demonstrates successful observer-controller integration where the Kalman filter estimates unknown gravity while the LQR controller computes optimal landing trajectories.