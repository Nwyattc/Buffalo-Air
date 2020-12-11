"""my_mavic_controller controller."""

from controller import Robot, Supervisor, Camera, Gyro, InertialUnit, GPS
import math

robot_node = None
supervisor = None
goal_location = None

def init_supervisor():
    global robot_node, supervisor, goal_location
    # create the Supervisor instance.
    supervisor = Supervisor()
    # do this once only
    root = supervisor.getRoot()
    root_children_field = root.getField("children")
    for idx in range(root_children_field.getCount()):
        name = root_children_field.getMFNode(idx).getTypeName()
        #print('name', name)
        if name == 'Mavic2Pro':
            robot_node = root_children_field.getMFNode(idx)
        if name == 'Solid':
            goal_node = root_children_field.getMFNode(idx)
            goal_location = goal_node.getField('translation').getSFVec3f()


def getMotorAll(robot):
    frontLeftMotor = robot.getMotor('front left propeller')
    frontRightMotor = robot.getMotor('front right propeller')
    backLeftMotor = robot.getMotor('rear left propeller')
    backRightMotor = robot.getMotor('rear right propeller')
    return [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor]


def motorsSpeed(robot, v1, v2, v3, v4):
    [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = getMotorAll(robot)
    frontLeftMotor.setVelocity(v1)
    frontRightMotor.setVelocity(-v2)
    backLeftMotor.setVelocity(-v3)
    backRightMotor.setVelocity(v4)
    return


def initializeMotors(robot):
    [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = getMotorAll(robot)
    frontLeftMotor.setPosition(float('inf'))
    frontRightMotor.setPosition(float('inf'))
    backLeftMotor.setPosition(float('inf'))
    backRightMotor.setPosition(float('inf'))
    speed = 0
    motorsSpeed(robot, speed, speed, speed, speed)
    return

def clamp(x, low, hi):
    return min(hi, max(low, x))

init_supervisor()
# create the Robot instance.
robot = supervisor

print("Goal Location:", goal_location)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialize camera
camera = Camera('camera')
camera.enable(1)
# position camera
camera_pitch_motor = robot.getMotor('camera pitch')
camera_pitch_motor.setPosition(math.pi/2)
# Start propellers
initializeMotors(robot)
prop_takeoff_speed = 68.5
motorsSpeed(robot, prop_takeoff_speed, prop_takeoff_speed, prop_takeoff_speed, prop_takeoff_speed)
# Initialize Gyro
myGyro = robot.getGyro('gyro')
myGyro.enable(timestep)
# Initialize InertialUnit
inertial_unit = InertialUnit('inertial unit')
inertial_unit.enable(timestep)
# Initialize GPS
gps = GPS('gps')
gps.enable(timestep)
#print("Coord Sys: ", gps.getCoordinateSystem())

# Initial Robot position
upright_roll = -math.pi / 2
upright_pitch = 0
upright_yaw = 0

# Initialize values for velocity calculation
last_altitude = 0
last_x = None
last_y = None

# Declare Target location from Supervisor call
target_x = goal_location[0]
target_y = goal_location[2]

state = "find target"
#state =  "landing"


###
# Main loop:
###
# - perform simulation steps until Webots stops the controller
while robot.step(timestep) != -1:
    print(" ") # For clarity between timesteps
    # GPS
    gps_x, altitude, gps_y = gps.getValues()

    # for altitude acceleration
    target_altitude = 1
    err_x = gps_x - target_x
    err_y = gps_y - target_y

    # for positional acceleration in terms of target
    if last_x is None:
        velocity_x = 0
    else:
        velocity_x = (gps_x - last_x) / timestep
    if last_y is None:
        velocity_y = 0
    else:
        velocity_y = (gps_y - last_y) / timestep
    last_x = gps_x
    last_y = gps_y
    vertical_velocity = (altitude - last_altitude) / timestep
    last_altitude = altitude

    # Gyro (acceleration)
    roll_acceleration, pitch_acceleration, yaw_acceleration = myGyro.getValues()
    # Rotation (radians)
    roll, pitch, yaw = inertial_unit.getRollPitchYaw()
    roll += math.pi / 2


    # Adjust motor speeds according to differences in actual and starting Roll/Pitch/Yaw
    # Scale and offset constants for tuning
    roll_scale = 50
    pitch_scale = 30
    vertical_scale = 3
    vertical_offset = .6
    vertical_velocity_scale = 5000
    xy_velocity_scale = 5000

    # Calculate inverse kinematics with a clamp() to have a Min and Max
    pitch_towards = -math.cos(yaw) * (err_x + xy_velocity_scale * velocity_x) + math.sin(yaw) * (err_y + xy_velocity_scale * velocity_y)
    roll_towards = math.sin(yaw) * (err_x + xy_velocity_scale * velocity_x) + math.cos(yaw) * (err_y + xy_velocity_scale * velocity_y)
    roll_disturbance = clamp(roll_towards, -0.5, 0.5)
    pitch_disturbance = clamp(pitch_towards, -0.5, 0.5)
    yaw_disturbance = 0.0 # Does not factor in to current algorithm, but potential next step

    # Factor in Gyro acceleration, Disturbance from IK, and scaled+clamped current roll/pitch/altitude
    roll_input = roll_scale * clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
    pitch_input = pitch_scale * clamp(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_difference_altitude = clamp(target_altitude - altitude + vertical_offset, -1.0, 1.0) # Clamp difference so it is not overbearing
    vertical_input = vertical_scale * (clamped_difference_altitude ** 3.0) # More scaling/tuning, with a power so it is less impactful the closer it is
    vertical_input -= vertical_velocity_scale * vertical_velocity # Account for velocity of drone from gps


    # Calculate total unit distance from the target
    total_xy_err = (err_x**2 + err_y**2)**.5
    # Display distance from target and current altitude
    print('Total Horizontal Error:', "%.5f" %total_xy_err, "meters")
    print("Altitude: ", "%.5f" %altitude, "meters")

    if total_xy_err < .20:
        state = 'landing'

    if state == 'landing' and altitude < 0.2: # Turn off motors once landed
        motorsSpeed(robot,0,0,0,0)
        break
    else:
        if state == 'landing':
            vertical_input = .35
            prop_takeoff_speed = 67 # Slow motors to lose altitude without falling too fast

        # Actuate the motors taking into consideration all the computed inputs.
        front_left_motor_speed = prop_takeoff_speed + vertical_input - roll_input - pitch_input + yaw_input
        front_right_motor_speed = prop_takeoff_speed + vertical_input + roll_input - pitch_input - yaw_input
        back_left_motor_speed = prop_takeoff_speed + vertical_input - roll_input + pitch_input - yaw_input
        back_right_motor_speed = prop_takeoff_speed + vertical_input + roll_input + pitch_input + yaw_input
        motorsSpeed(robot,
                    front_left_motor_speed,
                    front_right_motor_speed,
                    back_left_motor_speed,
                    back_right_motor_speed)


    # UN-USED FOR NOW
    # Image (3d matrix)
    # im = camera.getImage()

    ### Debugging Print Statements ###
    # print('Vertical Acceleration', vertical_velocity)
    # print('GPS data (altitude, x, y)', altitude, gps_x, gps_y)
    # print('Inertial Unit (roll, pitch, yaw):', roll, pitch, yaw)
    # print("x,y velocity", velocity_x, velocity_y)
    # print('Target Err (x,y)', err_x, err_y)

# Enter here exit cleanup code.
