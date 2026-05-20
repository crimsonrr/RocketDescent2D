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

# proportionality constant determines how aggressively the rocket should tilt in the opposite direction
tilt_limit = 0.40 # ~23 degrees
kp = 0.3 # Proportional controller: how much the gimbal should react when a tilt is noticed
kd = 0.15 # derivative of theta, looks at how fast the rocket is spinning and applies a counter force to slow the spinning down

engine_ignited = False

mass = 1.0
g = 9.81
I = 0.05

# log both dimensions to plot the rocket's flight path
x_history = []
y_history = []
# time_history = []

print("Starting Simulation...")

while y > 0.0:
    altitude_weight = np.clip(y / 100.0, 0.0, 1.0) # with this variable, the lower to the ground the rocket is, the less horizontal tilt adjustments are made
    target_theta = np.clip(-0.25 * v_x * altitude_weight, -tilt_limit, tilt_limit)  # forces the rocket to tilt in the opposite direction and fight the initial rightwards horizontal movement (v_x0 positive), if the rocket tilts too far to the left and ends up completely changing directions, the negatives cancel, and it will redirect itself.
    target_theta += -0.1 * theta
    error = theta - target_theta
    gimbal_angle = -(kp * error + kd * omega)
    gimbal_angle= np.clip(gimbal_angle, -0.26, 0.26, out=None)

# via kinematics, where the final velocity must be equal to zero
    accel_needed = -(v_y ** 2) / (2 * y)
# via f=ma
    max_thrust = 30.0
    accel_max = (max_thrust - (mass * g)) / mass

# when this occurs, the rocket has reached the absolute last second it has to break, therefore ignition begins
    if v_y < 0: # multiply by safety factor to account for some thrust lost trying to align itself horizontally
       effective_accel = accel_max * np.cos(theta + gimbal_angle)
       stopping_distance = (v_y ** 2) / (2 * effective_accel) # kinematics
    else:
        stopping_distance = 0
    if y <= stopping_distance * 2.0:
        engine_ignited = True
# dynamically throttle the engine
    if engine_ignited:
        target_theta *= 0.2

        k_v = 4.0
        k_p = 2.0
        desired_vy = -1.0 # soft landing desired
        vy_error = desired_vy - v_y

        thrust = mass * (g + k_v * vy_error)
        thrust = np.clip(thrust, 0.0, max_thrust)
    else:
        thrust = 0.0
    if y +v_y * dt <= 0:
        y = 0
        v_y = 0
        break

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