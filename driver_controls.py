from vex import *

# connect to: main.py
class controls:
    def __init__(self, controller, cata_motor, intake_motor, cata_direction = 0, intake_position = 0):
        self.controller = controller
        self.cata_direction = cata_direction
        self.intake_position = intake_position
        self.cata_motor = cata_motor
        self.intake_motor = intake_motor

    def x_enable_cata(self):

        if self.controller.buttonX.pressing():
            match self.cata_direction:
                case 0:
                    self.cata_motor.spin(FORWARD)

                case 1:
                    self.cata_motor.spin(REVERSE)

        else:
            self.cata_motor.stop()

    def b_toggle_intake(self):

        match self.intake_position:
            case 0:
                self.intake_position = 1
                self.intake_motor.spin_to_position(180, wait = False)

            case 1:
                self.intake_position = 0
                self.intake_motor.spin_to_position(0, wait = False)

    def update(self, **kargs):
        self.kargs = kargs

        if self.kargs["cata_direction"] == 0:
            self.cata_direction = 0

        elif self.kargs["cata_direction"] == 1:
            self.cata_direction = 1

        elif self.kargs["cata_direction"] == 2:
            match self.cata_direction:
                case 0:
                    self.cata_direction = 1

                case 1:
                    self.cata_direction = 0


        if self.kargs["intake_position"] == 0:
            self.intake_position = 0

        elif self.kargs["intake_position"] == 1:
            self.intake_position = 1

        elif self.kargs["intake_position"] == 2:
            match self.intake_position:
                case 0:
                    self.intake_position = 1

                case 1:
                    self.intake_position = 0