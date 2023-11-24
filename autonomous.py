# planing to connect to: main.py
class auto_track:
    def __init__(self, role):

        self.walls = [(0,0),(0,48),(48,48),(48,0)] # currently random values
        self.targets = [] #(x, y, angle)
        self.goal = []

        self.robotpos = (0, 0)
        self.robotang = 0
        self.curtarget = None
        
    def position_update(self, robotpos, robotang):
        self.robotpos = robotpos
        self.curtarget = robotang

    def pathfinding(self):
        pass