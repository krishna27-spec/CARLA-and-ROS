import subprocess
import time
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Generate Kp range from 0.5 to 30.0 with step 0.5
kp_values = np.arange(0.5, 30.5, 0.5)

print(f"Starting automated experiment sweep across {len(kp_values)} Kp values...")

for kp in kp_values:
    kp_str = f"{kp:.1f}"

    print(f"\n--- Running Experiment: Kp = {kp_str} ---")

    subprocess.run(["ros2", "service", "call", "/reset_simulation", "std_srvs/srv/Empty"])
    time.sleep(0.5)

    # Command to run ROS 2 node passing the 'kp' parameter
    cmd = [
        "ros2", "run", "controls_pkg", "p_controller",
        "--ros-args", "-p", f"kp:={kp}"
    ]

    # Execute and wait until the controller node completes its path and shuts down
    process = subprocess.Popen(cmd)
    try:
        # Hard cap: terminate process if it runs longer than 35 seconds
        process.wait(timeout=35.0)
    except subprocess.TimeoutExpired:
        print(f"Force killing Kp = {kp_str} due to timeout!")
        process.kill()
        process.wait()

    time.sleep(1)  # Brief pause between runs

print("\nAll experiments complete! Plotting results...")

# --- Plotting Module ---
csv_files = glob.glob("p_run_kp_*.csv")

fig, axes = plt.subplots(3, 1, figsize=(12, 10))

for file in sorted(csv_files):
    df = pd.read_csv(file)
    kp_val = file.replace("p_run_kp_", "").replace(".csv", "")

    # Filter out noisy lines or highlight key Kp milestones
    alpha_val = 0.8 if float(kp_val) in [0.5, 1.5, 5.0, 10.0, 20.0, 30.0] else 0.25

    # 1. Heading Error vs Time
    axes[0].plot(df['time'], df['heading_error'], label=f'Kp = {kp_val}', alpha=alpha_val)

    # 2. Vehicle Trajectory
    axes[1].plot(df['x'], df['y'], alpha=alpha_val)

    # 3. Angular Control Signal (Oscillation check)
    axes[2].plot(df['time'], df['omega'], alpha=alpha_val)

axes[0].set_title("Heading Error e(t) vs Time across Kp range [0.5 - 30.0]")
axes[0].set_ylabel("Heading Error (rad)")
axes[0].grid(True)

axes[1].set_title("Spatial Trajectory (X vs Y)")
axes[1].set_xlabel("X (m)")
axes[1].set_ylabel("Y (m)")
axes[1].grid(True)

axes[2].set_title("Angular Velocity Command (omega rad/s)")
axes[2].set_xlabel("Time (s)")
axes[2].set_ylabel("Omega (rad/s)")
axes[2].grid(True)

plt.tight_layout()
plt.savefig("p_controller_sweep_0.5_to_30.png")
plt.show()