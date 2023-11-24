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
intake_motor = Motor(Ports.PORT10, GearSetting.RATIO_36_1, False)


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

from odometry import Tracking # debugging
from driver_controls import controls # unfinished
from autonomous import auto_track # unfinished

# Begin project code    
class timer:
    def __init__(self):
        self.target_time = 0
    
    def set_time(self, timing):
        self.target_time = time.time_ns() + timing * (10**6)

    def check_time(self):
        if self.target_time <= time.time_ns():
            return True
        
        return False
    
# unit: inches
RDISTANCE = None
LDISTANCE = None
BDISTANCE = None
tracking = Tracking(LTrackWheel, RTrackWheel, BTrackWheel, 360 / (3.25 * math.pi), RDISTANCE, LDISTANCE, BDISTANCE) # inche per tick = 0.02836

inputs = controls(controller_1, Cata_Motor, intake_motor)
track_update_timer = timer()

# comp settings
def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

def driver_control():   
    brain.screen.clear_screen()
    brain.screen.print("driver control code")

    while True:

        if track_update_timer.check_time():
            
            tracking.update_encoder_values()

            # Display the cordinates for debugging 
            controller_1.screen.clear_screen()
            display_cord = tracking.get_cord() # adding this for readability
            controller_1.screen.print(f'Cordinates: ({round(display_cord[0], 2)}, {round(display_cord[1], 2)}), unit = Inches')
            controller_1.screen.next_row()
            controller_1.screen.print(f'Angle: {math.degrees(tracking.get_angle())} degrees')

            #add driver control assists (using odo)

            track_update_timer.set_time(10) # unit: miliseconds 

        inputs.x_enable_cata()  # enable cata when x is pressed
        controller_1.buttonB.pressed(inputs.b_toggle_intake)
        controller_1.buttonUp.pressed(lambda: inputs.update(cata_direction = 2))

comp = Competition(driver_control, autonomous)
