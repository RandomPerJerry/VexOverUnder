import math

# connect to: main.py
class Tracking:

    # length units: inches
    # angle units: radians

    def __init__(self, LTrackWheel, RTrackWheel, BTrackWheel, TICKSPERINCH, RDISTANCE, LDISTANCE, BDISTANCE):
        # Initialize variable using function peramerters
        self.LTrackWheel = LTrackWheel
        self.RTrackWheel = RTrackWheel
        self.BTrackWheel = BTrackWheel
        self.TICKSPERINCH = TICKSPERINCH
        self.RDISTANCE = RDISTANCE
        self.LDISTANCE = LDISTANCE
        self.BDISTANCE = BDISTANCE


        # Initialize encoder values
        self.curLval = 0
        self.curRval = 0
        self.curBval = 0
        # Initialize previous encoder values
        self.preLval = 0
        self.preRval = 0
        self.preBval = 0

        # Initialize delta values (change in encoder values)
        self.deltaL = 0
        self.deltaR = 0
        self.deltaS = 0

        # Initialize cord values
        self.preAngle = 0
        self.curAngle = 0
        self.curCord = (0, 0)

    def update_encoder_values(self):
        # Store the current encoder values in local variables
        self.preLval = self.curLval
        self.preRval = self.curRval
        self.preBval = self.curBval

        # Updating the values
        self.curLval = self.LTrackWheel.value()
        self.curRval = self.RTrackWheel.value()
        self.curBval = self.BTrackWheel.value()
        
        # Calculate the change in each encoders’ value since the last cycle, and convert to distance of wheel travel
        self.deltaL = (self.curLval - self.preLval) / self.TICKSPERINCH
        self.deltaR = (self.curRval - self.preRval) / self.TICKSPERINCH
        self.deltaS = (self.curBval - self.preBval) / self.TICKSPERINCH

    def get_angle(self):
        # Calculate new absolute orientation 
        # Formula: θ = (ΔL - ΔR) / (L + R)
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

        # Calculate global offset ( matrix rot2dation)
        # x' = x cos θ - y sin θ
        # y' = x sin θ + y cos θ
        globalOffset = [localOffset[0] * math.cos(-avgTheta) - localOffset[1] * math.sin(-avgTheta),
                        localOffset[0] * math.sin(-avgTheta) + localOffset[1] * math.cos(-avgTheta)]

        # Calculate new absolute position
        self.curCord = (self.curCord[0] + globalOffset[0], self.curCord[1] + globalOffset[1])

        return self.curCord
