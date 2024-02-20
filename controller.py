import numpy as np
import matplotlib.pyplot as plt

dt = 0.1
max_time = 500


def generate_signals():
    time = np.arange(0, max_time, dt)
    stable_signal = np.ones_like(time) * 0.5  # Stable synthetic signal
    sinusoidal_signal = np.sin(time) * 0.5 + 0.5  # Sinusoidal signal
    random_signal = np.random.rand(len(time))  # Random signal
    return time, stable_signal, sinusoidal_signal, random_signal


def pid_controller(input_signal, kp, ki, kd):
    output = np.zeros_like(input_signal)
    integral = 0
    prev_error = 0

    for i in range(len(input_signal)):
        error = setpoint_value - input_signal[i]
        integral += error * dt
        derivative = (error - prev_error) / dt
        control_signal = kp * error + ki * integral + kd * derivative
        output[i] = input_signal[i] + control_signal
        prev_error = error

    return output


def calculate_ultimate_gain_and_period(input_signal):
    max_output = max(input_signal)
    min_output = min(input_signal)
    amplitude = max_output - min_output
    half_amplitude = amplitude / 2

    for i in range(len(input_signal)):
        if input_signal[i] >= min_output + half_amplitude:
            start_time = i * dt
            break

    for i in range(len(input_signal) - 1, 0, -1):
        if input_signal[i] <= max_output - half_amplitude:
            end_time = i * dt
            break

    ultimate_period = end_time - start_time
    ultimate_gain = half_amplitude
    return ultimate_gain, ultimate_period


def tune_pid_parameters(input_signal):
    ultimate_gain, ultimate_period = calculate_ultimate_gain_and_period(
        input_signal)

    kp = 0.6 * ultimate_gain
    ki = 2 * kp / ultimate_period
    kd = kp * ultimate_period / 8

    return kp, ki, kd


setpoint_value = 0.5
time, stable_signal, sinusoidal_signal, random_signal = generate_signals()

kp_before, ki_before, kd_before = .5, .1, .05  # Initial PID parameters
kp_after, ki_after, kd_after = tune_pid_parameters(stable_signal)

output_stable_before = pid_controller(
    stable_signal, kp_before, ki_before, kd_before)
output_sinusoidal_before = pid_controller(
    sinusoidal_signal, kp_before, ki_before, kd_before)
output_random_before = pid_controller(
    random_signal, kp_before, ki_before, kd_before)

output_stable_after = pid_controller(
    stable_signal, kp_after, ki_after, kd_after)
output_sinusoidal_after = pid_controller(
    sinusoidal_signal, kp_after, ki_after, kd_after)
output_random_after = pid_controller(
    random_signal, kp_after, ki_after, kd_after)

plt.figure(figsize=(10, 12))

plt.subplot(3, 2, 1)
plt.plot(time, stable_signal, label='Stable Signal')
plt.plot(time, output_stable_before, label='Output Signal (Before Tuning)')
plt.title('PID Controller Response to Stable Signal (Before Tuning)')
plt.xlabel('Time')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True)

plt.subplot(3, 2, 3)
plt.plot(time, sinusoidal_signal, label='Sinusoidal Signal')
plt.plot(time, output_sinusoidal_before, label='Output Signal (Before Tuning)')
plt.title('PID Controller Response to Sinusoidal Signal (Before Tuning)')
plt.xlabel('Time')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True)

plt.subplot(3, 2, 5)
plt.plot(time, random_signal, label='Random Signal')
plt.plot(time, output_random_before, label='Output Signal (Before Tuning)')
plt.title('PID Controller Response to Random Signal (Before Tuning)')
plt.xlabel('Time')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True)

# Plot results after tuning
plt.subplot(3, 2, 2)
plt.plot(time, stable_signal, label='Stable Signal')
plt.plot(time, output_stable_after, label='Output Signal (After Tuning)')
plt.title('PID Controller Response to Stable Signal (After Tuning)')
plt.xlabel('Time')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True)

plt.subplot(3, 2, 4)
plt.plot(time, sinusoidal_signal, label='Sinusoidal Signal')
plt.plot(time, output_sinusoidal_after, label='Output Signal (After Tuning)')
plt.title('PID Controller Response to Sinusoidal Signal (After Tuning)')
plt.xlabel('Time')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True)

plt.subplot(3, 2, 6)
plt.plot(time, random_signal, label='Random Signal')
plt.plot(time, output_random_after, label='Output Signal (After Tuning)')
plt.title('PID Controller Response to Random Signal (After Tuning)')
plt.xlabel('Time')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
