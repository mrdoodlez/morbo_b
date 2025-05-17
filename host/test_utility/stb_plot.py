#!/usr/bin/env python3

import matplotlib.pyplot as plt

# Define log file name
log_file = "log.txt"  # Change to your actual log path

# Data arrays
roll = []
pitch = []
yaw = []
roll_rate = []
pitch_rate = []
yaw_rate = []
pid_roll = []
pid_pitch = []
pid_yaw = []
thrust1 = []
thrust2 = []
thrust3 = []
thrust4 = []

# Load and parse log
total_lines = 0
with open(log_file, 'r') as f:
    for line in f:
        if line.startswith("STB:"):
            parts = line.strip().split()[1:]  # remove "STB:"
            values = list(map(float, parts))
            if len(values) == 13:
                roll.append(values[0])
                pitch.append(values[1])
                yaw.append(values[2])
                roll_rate.append(values[3])
                pitch_rate.append(values[4])
                yaw_rate.append(values[5])
                pid_roll.append(values[6])
                pid_pitch.append(values[7])
                pid_yaw.append(values[8])
                thrust1.append(values[9])
                thrust2.append(values[10])
                thrust3.append(values[11])
                thrust4.append(values[12])
                total_lines += 1

print(f"Parsed {total_lines} STB log entries.")

# Compute derived values
roll_thrust_diff = [(t1 + t2) - (t3 + t4) for t1, t2, t3, t4 in zip(thrust1, thrust2, thrust3, thrust4)]
pitch_thrust_diff = [(t2 + t4) - (t1 + t3) for t1, t2, t3, t4 in zip(thrust1, thrust2, thrust3, thrust4)]

# Time axis
t = list(range(len(roll)))

# Plot
fig, axs = plt.subplots(8, 1, figsize=(10, 16), sharex=True)

axs[0].plot(t, roll, label='roll', color='r')
axs[0].set_ylabel('Roll (rad)')
axs[0].legend()
axs[0].grid()

axs[1].plot(t, roll_rate, label='roll_rate', linestyle='--', color='b')
axs[1].set_ylabel('Roll Rate (rad/s)')
axs[1].legend()
axs[1].grid()

axs[2].plot(t, pid_roll, label='pid_roll', color='g')
axs[2].set_ylabel('PID Roll')
axs[2].legend()
axs[2].grid()

axs[3].plot(t, roll_thrust_diff, label='(T1+T2)-(T3+T4)', color='m')
axs[3].set_ylabel('Roll Thrust Diff')
axs[3].legend()
axs[3].grid()

axs[4].plot(t, pitch, label='pitch', color='r')
axs[4].set_ylabel('Pitch (rad)')
axs[4].legend()
axs[4].grid()

axs[5].plot(t, pitch_rate, label='pitch_rate', linestyle='--', color='b')
axs[5].set_ylabel('Pitch Rate (rad/s)')
axs[5].legend()
axs[5].grid()

axs[6].plot(t, pid_pitch, label='pid_pitch', color='g')
axs[6].set_ylabel('PID Pitch')
axs[6].legend()
axs[6].grid()

axs[7].plot(t, pitch_thrust_diff, label='(T2+T4)-(T1+T3)', color='c')
axs[7].set_ylabel('Pitch Thrust Diff')
axs[7].set_xlabel('Sample Index')
axs[7].legend()
axs[7].grid()

plt.tight_layout()
plt.show()
