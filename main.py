import numpy as np
import matplotlib.pyplot as plt

# VARIABLES
t = 0.0
dt = 0.005

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
kd = 0.15 # modifies the velocity (d/dx), looks at how fast the rocket is spinning and applies a counter force to slow the spinning down

mass = 1.0
g = 9.81
I = 0.05
# log both dimensions to plot the rocket's flight path
x_history = []
y_history = []
# time_history = []

print("Starting Simulation...")

while y > 0.0:
    max_thrust = 30.0
    accel_max = (max_thrust - (mass * g)) / mass
    v_safe = np.sqrt(2 * accel_max * max(y, 0))
    flare_altitude = 30.0  # at this point, the rocket must increase throttle as it is nearing the ground
    if y > flare_altitude:
        k = 0.5
        desired_vx = -k * x  # direct the velocity to the opposite direction
        desired_vy = -min(v_safe, 5.0)
        if v_y < desired_vy:
            ay_desired = accel_max # allows the rocket "catch up" and actually achieve the v_safe velocity

        vx_error = desired_vx - v_x
        vy_error = desired_vy - v_y
        ky = 1.5
        kx = 0.5
        ay_desired = ky * (desired_vy - v_y) + 0.5 * (0 - y)
        ay_desired = np.clip(ay_desired, -accel_max, accel_max)
        ax_desired = kx * vx_error
    else:
    # the desired velocities at this point are 0 (rest)
        target_vy = -v_safe * (y / flare_altitude) # dynamically calculate the target velocity, which eventually goes to zero or rest
        target_vy = max(target_vy, -v_safe)
        vy_error = target_vy - v_y

        kpy = 2.0
        kdy = 3.0
        ay_desired = kdy * (target_vy - v_y)

        kpx = 1.0
        kdx = 2.0
        ax_desired = kpx * (0 - x) + kdx * (0 - v_x)

    altitude_weight = np.clip(y / 100.0, 0.0,1.0)  # with this variable, the lower to the ground the rocket is, the less horizontal tilt adjustments are made
    theta_target = np.clip((ax_desired / g) * altitude_weight, -tilt_limit, tilt_limit)  # forces the rocket to tilt in the opposite direction and fight the initial rightwards horizontal movement (v_x0 positive), if the rocket tilts too far to the left and ends up completely changing directions, the negatives cancel, and it will redirect itself.
    theta_target += -0.1 * theta

    theta_error = theta_target - theta
# let the gimbal control rotation *only*
    gimbal_angle = kp * theta_error - kd * omega
    gimbal_angle = np.clip(gimbal_angle, -0.26, 0.26, out=None)

    thrust = mass * g + mass * ay_desired + 5.0 # add margin
    thrust = np.clip(thrust, 0.0, max_thrust)

# initiate full thrust if the rocket is still traveling too fast near the ground
    if y > 1.0 and abs(v_y) > v_safe:
        thrust = max_thrust

# the direction the engine pushes is a combination of the rocket body's tilt and the engine's gimbal tilt
# let theta control the direction of the thrust
    forcex = thrust * np.sin(theta)
    forcey = (thrust * np.cos(theta)) - (mass * g)

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

    print("VELOCITY LOGS:", "y:", y, "v_y:", v_y, "v_safe:", v_safe)
    print("ACCELERATION LOGS:", "ay_desired:", ay_desired, "max accel:", accel_max)

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