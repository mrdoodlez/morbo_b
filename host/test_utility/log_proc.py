#!/usr/bin/env python3

import pandas as pd
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# Replace these with your actual ADC samples
adc_0g = np.array([0.088420, 0.090829, 0.089442, 0.092397, 0.088440, 0.085134, 0.084440, 0.091626, 0.094281, 0.088239, 0.091683, 0.091659, 0.087114, 0.087912, 0.086673])
adc_100g = np.array([0.299298, 0.298221, 0.300083, 0.300738, 0.296155, 0.295649, 0.295815, 0.297933, 0.295756, 0.299635, 0.294286, 0.297836, 0.298910, 0.298596, 0.296931])
adc_1000g = np.array([2.360642, 2.362251, 2.363986, 2.362926, 2.365704, 2.361459, 2.364877, 2.369610, 2.364363, 2.362485, 2.361284, 2.366976, 2.370442, 2.362569, 2.362433])

L1 = 94.0
L2 = 42.5

# Step 1: Compute means
adc_means = np.array([
    np.mean(adc_0g),
    np.mean(adc_100g),
    np.mean(adc_1000g)
])
weights = np.array([0, 100, 1000])  # grams

# Step 2: Fit a linear model: weight = a * adc + b
def linear(adc, a, b):
    return a * adc + b

params, _ = curve_fit(linear, adc_means, weights)
a, b = params
print(f"Calibration equation: weight = {a:.6f} * adc + {b:.6f}")

'''
# Step 3: Plot calibration
adc_range = np.linspace(adc_means.min(), adc_means.max(), 100)
weight_pred = linear(adc_range, a, b)

plt.scatter(adc_means, weights, label="Calibration points")
plt.plot(adc_range, weight_pred, 'r-', label="Fitted line")
plt.xlabel("ADC Value")
plt.ylabel("Weight (grams)")
plt.title("Weight Sensor Calibration")
plt.legend()
plt.grid(True)
plt.show()
'''

def quad_model(thrust, a, b, c):
    return a * thrust**2 + b * thrust + c

def adc_to_grams(adc_value):
    return a * adc_value + b

# --- Parse MON logs ---
def parse_mon_logs_with_thrust(file_path):
    records = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('MON:'):
                content = line.split(':', 1)[1].strip()
                values = list(map(float, content.split()))
                records.append(values)

    df = pd.DataFrame(records, columns=[
        'pwm1', 'pwm2', 'pwm3', 'pwm4', 'battery_voltage', 'adc_value'
    ])

    df['pwm_avg'] = df[['pwm1', 'pwm2', 'pwm3', 'pwm4']].mean(axis=1)
    df['motor_voltage'] = df['pwm_avg'] * df['battery_voltage']
    df['thrust_grams'] = L1 / L2 * adc_to_grams(df['adc_value']) / 4.0

    return df

# Example
df = parse_mon_logs_with_thrust('log.txt')
print(df[['motor_voltage', 'thrust_grams']].head())

df = parse_mon_logs_with_thrust('log.txt')

# Step 1: Compute average PWM and total motor voltage
df['pwm_avg'] = df[['pwm1', 'pwm2', 'pwm3', 'pwm4']].mean(axis=1)
df['motor_voltage'] = df['pwm_avg'] * df['battery_voltage']

# Step 2: Visualize relationship
plt.scatter(df['motor_voltage'], df['thrust_grams'], alpha=0.5)
plt.xlabel('Motor Voltage (V)')
plt.ylabel('Thrust per motor (g)')
plt.title('Thrust vs Motor Voltage')
plt.grid(True)
plt.show()

x = df['thrust_grams'].values  # INPUT is thrust
y = df['motor_voltage'].values  # OUTPUT is required motor voltage

popt, _ = curve_fit(quad_model, x, y)

# Print the model
print(f"motor_voltage = {popt[0]:.3e} * thrust² + {popt[1]:.3e} * thrust + {popt[2]:.3e}")

x_fit = np.linspace(x.min(), x.max(), 500)
y_fit = quad_model(x_fit, *popt)

plt.scatter(x, y, alpha=0.4, label='Data')
plt.plot(x_fit, y_fit, 'r-', label='Fitted model')
plt.xlabel('Target Thrust(g)')
plt.ylabel('Required Motor Voltage (V)')
plt.grid(True)
plt.legend()
plt.title('Motor Voltage vs Target Thrust')
plt.show()


