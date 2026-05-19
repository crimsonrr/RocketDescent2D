import numpy as np
import matplotlib.pyplot as plt

# VARIABLES
t = 0.0
dt = 0.01

v_x = 2.0
x = 0.0
v_y = 0.0
y = 100.0

theta = 0.08 # roughly 5 degrees
omega = 0.0

gimbal_angle = 0.02 # radians (The angle the engine nozzle tilts relative to the rocket body)
l_thrust = 2.0 # distance from the engine nozzle to the rocket's COM  (m)

target_theta = 0.0 # desired angle is 0, (equilibrium)
kp = 0.05 # Proportional controller: how much the gimbal should react when a tilt is noticed
kd = 0.05 # derivative of theta, looks at how fast the rocket is spinning and applies a counter force to slow the spinning down

mass = 1.0
g = 9.81
I = 0.05

# log both dimensions to plot the rocket's flight path
x_history = []
y_history = []
# time_history = []

print("Starting Simulation...")

while y > 0.0:
    error = theta - target_theta
    gimbal_angle = -(kp * error + kd * omega)
    gimbal_angle= np.clip(gimbal_angle, -0.26, 0.26, out=None)

    if y < 80.0:
        thrust = 12.0
    else:
        thrust = 0.0

# the direction the engine pushes is a combination of the rocket body's tilt and the engine's gimbal tilt
    forcex = thrust * np.sin(theta + gimbal_angle)
    forcey = (thrust * np.cos(theta + gimbal_angle)) - (mass * g)

    accelerationx = forcex / mass
    accelerationy = forcey / mass

# linear euler integration
    v_x = v_x + (accelerationx * dt)
    v_y = v_y + (accelerationy * dt)
    x = x + (v_x * dt)
    y = y + (v_y * dt)

# rotational euler integration
    tau = -thrust * np.sin(gimbal_angle) * l_thrust
    alpha = tau / I
    omega = omega + (alpha * dt)
    theta = theta + (omega * dt)

# log the data
    x_history.append(x)
    y_history.append(y)
    t += dt

print("Rocket Landed/Crashed")

plt.figure(figsize=(6, 8))
plt.plot(x_history, y_history, label="Flight Path")
plt.axhline(0, color='black', linestyle='--') # ground line
plt.xlabel('Horizontal Position X (m)')
plt.ylabel('Vertical Altitude Y (m)')
plt.title('2D Rocket Trajectory')
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box') # keeps scale 1:1
plt.show()