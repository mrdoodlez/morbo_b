#!/usr/bin/env python3

import matplotlib.pyplot as plt
import math
from scipy.optimize import fsolve
import pandas as pd

# Параметры
m = 2  # модуль
z_motor = 13
z_stage1 = 100
z_stage2 = 14
z_out = 91

# Функции расчёта геометрии шестерён
def gear_dims(z, m):
    d = m * z
    da = d + 2 * m
    df = d - 2.5 * m
    return round(d, 2), round(da, 2), round(df, 2)

# Размеры
motor_d, motor_da, motor_df = gear_dims(z_motor, m)
stage1_d, stage1_da, stage1_df = gear_dims(z_stage1, m)
stage2_d, stage2_da, stage2_df = gear_dims(z_stage2, m)
out_d, out_da, out_df = gear_dims(z_out, m)

# Межосевые расстояния
a_motor_stage1 = (motor_d + stage1_d) / 2
a_stage1_out = (stage2_d + out_d) / 2

R1 = a_motor_stage1
R2 = a_stage1_out

# Решение планиметрии
def solve_planimetry(y_out_input):
    def equations(vars):
        x_out, x_i, y_i = vars
        eq1 = (x_i - 0)**2 + (y_i - 0)**2 - R1**2
        eq2 = (x_i - x_out)**2 + (y_i - y_out_input)**2 - R2**2
        eq3 = x_out - 100
        return [eq1, eq2, eq3]

    initial_guess = [200, R1 / 2, -R1 / 2]
    solution = fsolve(equations, initial_guess)
    x_out_sol, x_i_sol, y_i_sol = solution

    res = equations(solution)
    if max(abs(r) for r in res[:2]) > 1e-3:
        return None, "⚠️ Нет точного решения"

    coords = {
        'Motor': (0.0, 0.0),
        'Intermediate': (round(x_i_sol, 2), round(y_i_sol, 2)),
        'Output': (round(x_out_sol, 2), round(y_out_input, 2))
    }
    return coords, "✅ Решение найдено"

# Функция для отрисовки
def solve_and_draw(y_out_input):
    coords_planimetry, result_text = solve_planimetry(y_out_input)

    fig, ax = plt.subplots(figsize=(10, 8))

    motor_circle = plt.Circle((0, 0), motor_da / 2, color='red', fill=False, linewidth=2)
    ax.add_artist(motor_circle)
    ax.text(0, 0, f"Motor\nZ={z_motor}\nØ{motor_da:.1f}", ha='center', va='center', fontsize=9)

    if coords_planimetry:
        x_out_sol = coords_planimetry['Output'][0]
        out_circle = plt.Circle((x_out_sol, y_out_input), out_da / 2, color='blue', fill=False, linewidth=2)
        ax.add_artist(out_circle)
        ax.text(x_out_sol, y_out_input, f"Output\nZ={z_out}\nØ{out_da:.1f}", ha='center', va='center', fontsize=9)

        xi, yi = coords_planimetry['Intermediate']
        inter_circle_stage1 = plt.Circle((xi, yi), stage1_da / 2, color='green', fill=False, linewidth=2)
        inter_circle_stage2 = plt.Circle((xi, yi), stage2_da / 2, color='orange', fill=False, linewidth=2)
        ax.add_artist(inter_circle_stage1)
        ax.add_artist(inter_circle_stage2)
        ax.text(xi, yi, f"Intermediate\nZ1={z_stage1}\nZ2={z_stage2}", ha='center', va='center', fontsize=9)

    ax.set_aspect('equal')
    ax.set_xlim(-100, 350)
    ax.set_ylim(-300, 150)
    plt.grid(True)
    plt.title(f"Gearbox layout — full planimetry\ny_out = {y_out_input} mm\n{result_text}")

    plt.show()

    # Таблица параметров + координат
    if coords_planimetry:
        df_dims = pd.DataFrame([
            ['Motor', z_motor, motor_d, motor_da, motor_df, coords_planimetry['Motor'][0], coords_planimetry['Motor'][1]],
            ['Intermediate (stage1)', z_stage1, stage1_d, stage1_da, stage1_df, coords_planimetry['Intermediate'][0], coords_planimetry['Intermediate'][1]],
            ['Intermediate (stage2)', z_stage2, stage2_d, stage2_da, stage2_df, coords_planimetry['Intermediate'][0], coords_planimetry['Intermediate'][1]],
            ['Output', z_out, out_d, out_da, out_df, coords_planimetry['Output'][0], coords_planimetry['Output'][1]]
        ], columns=['Gear', 'Z (teeth)', 'Pitch diameter d (mm)', 'Outer diameter da (mm)', 'Root diameter df (mm)', 'Center X (mm)', 'Center Y (mm)'])

        print(df_dims)

if __name__ == "__main__":
    # Вызов функции с нужным параметром
    solve_and_draw(-170)