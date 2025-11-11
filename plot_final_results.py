#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

def plot_final_results():
    try:
        # Read the data
        data = np.loadtxt('FINAL_LANDER.txt')
        
        time = data[:, 0]
        true_h = data[:, 1]
        true_v = data[:, 2]
        est_h = data[:, 3]
        est_v = data[:, 4]
        est_g = data[:, 5]
        control = data[:, 6]
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Height plot
        axes[0, 0].plot(time, true_h, 'b-', linewidth=2, label='True Height')
        axes[0, 0].plot(time, est_h, 'r--', linewidth=2, label='Estimated Height')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Height (m)')
        axes[0, 0].set_title('Lander Height During Descent')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        axes[0, 0].set_ylim(bottom=0)
        
        # Velocity plot
        axes[0, 1].plot(time, true_v, 'g-', linewidth=2, label='True Velocity')
        axes[0, 1].plot(time, est_v, 'm--', linewidth=2, label='Estimated Velocity')
        axes[0, 1].axhline(y=0, color='k', linestyle=':', alpha=0.7)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].set_title('Lander Velocity During Descent')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Gravity estimation
        axes[1, 0].plot(time, est_g, 'orange', linewidth=2, label='Estimated Gravity')
        axes[1, 0].axhline(y=5.5, color='r', linestyle='--', linewidth=2, label='True Gravity (5.5 m/s²)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Gravity (m/s²)')
        axes[1, 0].set_title('Kalman Filter Gravity Estimation')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # Control input
        axes[1, 1].plot(time, control, 'purple', linewidth=2, label='Control Input')
        axes[1, 1].axhline(y=0, color='k', linestyle=':', alpha=0.7)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Control Force (N)')
        axes[1, 1].set_title('LQR Controller Output')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('lander_mission_results.png', dpi=300, bbox_inches='tight')
        print("Mission results plot saved as 'lander_mission_results.png'")
        
        # Summary statistics
        print(f"\n=== MISSION ANALYSIS ===")
        print(f"Mission duration: {time[-1]:.1f} seconds")
        print(f"Landing velocity: {true_v[-1]:.1f} m/s")
        print(f"Final gravity estimate: {est_g[-1]:.2f} m/s²")
        print(f"Gravity estimation error: {abs(est_g[-1] - 9.81):.2f} m/s²")
        
        # Check convergence
        if len(est_g) > 60:  # After 2 seconds
            converged_gravity = np.mean(est_g[-30:])  # Last 1 second average
            print(f"Converged gravity estimate: {converged_gravity:.2f} m/s²")
            
        plt.show()
        
    except FileNotFoundError:
        print("FINAL_LANDER.txt not found. Run ./final_lander first.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    plot_final_results()