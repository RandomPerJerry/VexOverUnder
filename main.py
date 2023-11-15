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

PI = math.pi
sin = math.sin
rad = math.radians
class Tracking:

    #the distance btween each wheel and the tracking center 
    LDISTANCE = None
    RDISTANCE = None
    BDISTANCE = None
    WHEELLENGTH = 3 #Diameter in inch
    TICKSPERINCH = 360 / (WHEELLENGTH * PI)


    def __init__(self):
        #storing the pervious values of encorder to calculate the change 
        self.preLval = 0
        self.preRval = 0
        self.preBval = 0

        #storing the current values of encoder
        self.curLval = LTrackWheel.value()
        self.curRval = RTrackWheel.value()
        self.curBval = BTrackWheel.value()

        #The current recorded cordinates and angle 
        self.curCord = (0, 0)
        self.curAngle = 0

    def update_encoder_values(self):
        #storing the value into the prev version before updating the current values
        self.preLval = self.curLval
        self.curRval = self.curRval
        self.curBval = self.curBval

        #updating the values
        self.curLval = LTrackWheel.value()
        self.curRval = RTrackWheel.value()
        self.curBval = BTrackWheel.value()

    def get_angle(self):
        #calculating the change in angle, Formula: dAngle = (left arc lenth - right arc lenth) / (RDISTANCE + LDISTANCE)
        dAngle = ((self.curLval - self.preLval - self.curRval + self.preRval) / self.TICKSPERINCH) / (self.RDISTANCE + self.LDISTANCE)
        self.curAngle += dAngle

        return self.curAngle

    def get_cord(self):
        # Formula: x = 2 * sin(theta/2) * ((back arc lenth / theta)+BDISTANCE)
        # Formula: y = 2 * sin(theta/2) * ((right arc lenth / theta)+RDISTANCE)

        x_cord = 2 * sin(rad(self.get_angle() / 2)) * (((self.curBval - self.preBval) / self.TICKSPERINCH) / self.get_angle() + self.BDISTANCE)
        y_cord = 2 * sin(rad(self.get_angle() / 2)) * (((self.curRval - self.preRval) / self.TICKSPERINCH) / self.get_angle() + self.RDISTANCE)

        self.curCord = (x_cord, y_cord)






