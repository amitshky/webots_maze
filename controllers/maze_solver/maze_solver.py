"""maze solver controller"""

# This file contains only a bare-bones structure with initialization of variables,
# sensor reading, a few functions and a simple state machine to solve a maze.
# You can use it as a template for your solution.

# Author: Felipe N. Martins
# 05-MAR-2024

from controller import Robot, DistanceSensor, Motor, Compass
import numpy as np

# -------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = robot.getBasicTimeStep()/1000.0    # [s]

# states for the state machine: modify according to your state machine
states = ['follow_wall', 'turn_cw', 'turn_acw']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 100

# Robot wheel speeds: initial values are zero
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds: initial values are zero
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0520    # distance between the wheels: 52mm [m]

# -------------------------------------------------------
# Initialize devices

# proximity sensors: measure distance to walls and obstacles
ps = []
ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(ps_names[i]))
    ps[i].enable(timestep)

# encoders: measure the angular position of the wheels in radians
encoder = []
encoder_names = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoder_names[i]))
    encoder[i].enable(timestep)

old_encoder_vals = []

# camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# -------------------------- Functions ------------------------------------
# Complete the functions below and create new ones to complete the mission


def get_wheels_speed(encoder_vals, old_encoder_vals, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    wl = (encoder_vals[0] - old_encoder_vals[0]) / delta_t
    wr = (encoder_vals[1] - old_encoder_vals[1]) / delta_t

    return wl, wr


def get_robot_speeds(wl, wr, R, D):
    """Computes robot linear and angular speeds"""
    u = (R / 2.0) * (wr + wl)
    w = (R / D) * (wr - wl)

    return u, w


def wheel_speed_commands(u_d, w_d, D, R):
    """Converts desired speeds to wheel speed commands
    Inputs:
        u_d = desired linear speed for the robot [m/s]
        w_d = desired angular speed for the robot [rad/s]
        R = radius of the robot wheel [m]
        D = distance between the left and right wheels [m]
    Returns:
        wr_d = desired speed for the right wheel [rad/s]
        wl_d = desired speed for the left wheel [rad/s]
    """

    wr_d = (2.0 * u_d + D * w_d) / (2.0 * R)
    wl_d = (2.0 * u_d - D * w_d) / (2.0 * R)

    return wl_d, wr_d


#######################################
# Write here additional functions here
#######################################


# ---------------------------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    left_speed = 0
    right_speed = 0
    # Implement the see-think-act cycle:

    ############################################
    #                   See                    #
    ############################################

    # Sensor values are being placed in the corresponding lists.
    # proximity sensors:
    ps_vals = []
    for i in range(8):
        ps_vals.append(ps[i].getValue())
    # encoders:
    encoder_vals = []
    for i in range(2):
        encoder_vals.append(encoder[i].getValue())    # [rad]
    # Update old encoder values if not done before
    if len(old_encoder_vals) < 2:
        for i in range(2):
            old_encoder_vals.append(encoder[i].getValue())

    # Add code here for the camera
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    ############################################
    #                  Think                   #
    ############################################

    # Implement the state machine to select proper behaviors
    left_obstacle = ps_vals[7] > 80.0 or ps_vals[6] > 80.0
    right_obstacle = ps_vals[0] > 80.0 or ps_vals[1] > 80.0
    if left_obstacle:
        left_speed = 0.8 * MAX_SPEED
        right_speed = 0.2 * MAX_SPEED
    elif right_obstacle:
        left_speed = 0.2 * MAX_SPEED
        right_speed = 0.8 * MAX_SPEED
    else:
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED

    red = 0
    green = 0
    blue = 0
    count = 0
    for x in range(width // 3, 2 * width // 3):
        for y in range(height // 3, 2 * height // 3):
            count += 1
            red += camera.imageGetRed(image, width, x, y)
            green += camera.imageGetGreen(image, width, x, y)
            blue += camera.imageGetBlue(image, width, x, y)

    red //= count
    green //= count
    blue //= count
    is_wall_ahead = False
    if red > 200 and green > 200 and blue > 200:
        is_wall_ahead = True

    # increment counter
    counter += 1

    # update old encoder values for the next cycle
    old_encoder_vals = encoder_vals

    ############################################
    #                   Act                    #
    ############################################
    # Set motor speeds with the values defined by the state machine.
    # This part is already working. You only need to complete the function.

    # left_speed, right_speed = wheel_speed_commands(u_d, w_d, D, R)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # To help on debugging:
    print(f'Current state = {current_state}, Wall Ahead: {is_wall_ahead}')
    print(f"Front ps:      {ps_vals[7]}, {ps_vals[0]}")
    print(f"Front-side ps: {ps_vals[6]}, {ps_vals[1]}")
    print(f"Side ps:       {ps_vals[5]}, {ps_vals[2]}")
    print(f"Back ps:       {ps_vals[4]}, {ps_vals[3]}")

    # End of the loop. Repeat all steps while the simulation is running.
