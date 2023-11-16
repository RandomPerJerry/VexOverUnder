#region VEXcode Generated Robot Configuration
from vex import *

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
left_motor_a = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
Cata_Motor_motor_a = Motor(Ports.PORT5, GearSetting.RATIO_36_1, True)
Cata_Motor_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_36_1, False)
Cata_Motor = MotorGroup(Cata_Motor_motor_a, Cata_Motor_motor_b)
LTrackWheel = Encoder(brain.three_wire_port.a)
RTrackWheel = Encoder(brain.three_wire_port.c)
BTrackWheel = Encoder(brain.three_wire_port.e)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")



# define variables used for controlling motors based on controller inputs
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = controller_1.axis3.position() + controller_1.axis1.position()
            drivetrain_right_side_speed = controller_1.axis3.position() - controller_1.axis1.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *
import math
import time

# Begin project code

class Tracking:

    def __init__(self, LTrackWheel, RTrackWheel, BTrackWheel, TICKSPERINCH, RDISTANCE, LDISTANCE, BDISTANCE):
        # Initialize variable using function peramerters
        self.LTrackWheel = LTrackWheel
        self.RTrackWheel = RTrackWheel
        self.BTrackWheel = BTrackWheel
        self.TICKSPERINCH = TICKSPERINCH
        self.RDISTANCE = RDISTANCE
        self.LDISTANCE = LDISTANCE
        self.BDISTANCE = BDISTANCE

        self.preAngle = 0
        self.curAngle = 0
        self.curCord = (0, 0)

    def update_encoder_values(self):
        # Store the current encoder values in local variables
        self.preLval = self.curLval
        self.preRval = self.curRval
        self.preBval = self.curBval

        # Updating the values
        self.curLval = LTrackWheel.value()
        self.curRval = RTrackWheel.value()
        self.curBval = BTrackWheel.value()

        # Calculate the change in each encoders’ value since the last cycle, and convert to distance of wheel travel
        self.deltaL = (self.curLval - self.preLval) / self.TICKSPERINCH
        self.deltaR = (self.curRval - self.preRval) / self.TICKSPERINCH
        self.deltaS = (self.curBval - self.preBval) / self.TICKSPERINCH

    def get_angle(self):
        # Calculate new absolute orientation 
        # Formula: θ = θ + (ΔL - ΔR) / (L + R)
        self.curAngle += (self.deltaL - self.deltaR) / (self.LDISTANCE + self.RDISTANCE)
        return self.curAngle

    def get_cord(self):
        # Calculate the change in angle
        deltaTheta = self.get_angle() - self.preAngle
        self.preAngle = self.get_angle()

        # Calculate the local offset
        # Formula: X: 2 sin(Δθ / 2) * (ΔS / Δθ + B)
        #          Y: 2 sin(Δθ / 2) * (ΔR / Δθ + R)
        if deltaTheta == 0:
            localOffset = [self.deltaS, self.deltaR]
        else:
            localOffset = [2 * math.sin(deltaTheta / 2) * (self.deltaS / deltaTheta + self.BDISTANCE),
                           2 * math.sin(deltaTheta / 2) * (self.deltaR / deltaTheta + self.RDISTANCE)]

        # Calculate the average orientation
        avgTheta = self.preAngle + deltaTheta / 2

        # Calculate global offset (2d matrix rotation)
        # x' = x cos θ - y sin θ
        # y' = x sin θ + y cos θ
        globalOffset = [localOffset[0] * math.cos(-avgTheta) - localOffset[1] * math.sin(-avgTheta),
                        localOffset[0] * math.sin(-avgTheta) + localOffset[1] * math.cos(-avgTheta)]

        # Calculate new absolute position
        self.curCord = (self.curCord[0] + globalOffset[0], self.curCord[1] + globalOffset[1])

        return self.curCord


# Create a Tracking object
tracking = Tracking(LTrackWheel, RTrackWheel, BTrackWheel, TICKSPERINCH, RDISTANCE, LDISTANCE, BDISTANCE)

#   TICKSPERINCH = 360 / (x * PI) # x is the diameter of the wheel in inch
#   RDISTANCE # distance from the center of the robot to the right wheel
#   LDISTANCE # distance from the center of the robot to the left wheel
#   BDISTANCE # distance from the center of the robot to the back wheel






