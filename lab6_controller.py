from controller import Compass, Robot, DistanceSensor, Motor
import math
import re

class Optimizer:
    def __init__(self):
        self.filePath = "./data.txt"
        self.file = self.read_in()
        self.data = []
        if self.doesFileExist():
            temp = self.file.readlines()
            for l in temp:
                self.data.append(l.strip('\n'))

    def read_in(self):
        try:
            file = open(self.filePath, "r")
            pass
        except:
            file = 0
        return file

    def doesFileExist(self):
        return not self.file == 0

    def addData(self, data):
        self.data.append(data)

    def checkPoints(self, start, end):
        pass

    def writeData(self):
        with open(self.filePath, "w") as f:
            for d in self.data:
                self.file.write("%s\n" % d)
            f.close()

    def writeData(self, data):
        with open(self.filePath, "a") as f:
            f.write("%s\n" % data)
            f.close()

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

DETECT_VAL = 100.0

#Rotate 2D vector
def rotate(vector, angle):
    temp = [0, 0]
    angle = math.radians(angle)
    temp[0] = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
    temp[1] = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
    return temp

#Return if is straight based on N,S,E,W
#angle = float, direction = cardinal direction
def isStraight(angle, direction):
    if direction == 'N':
        return angle > 85 and angle < 95
    elif direction == 'E':
        return angle > 175 or angle < -175
    elif direction == 'S':
        return angle > -95 and angle < -85
    elif direction == 'W':
        return angle > -5 and angle < 5
    else:
        print("Incorrection direction")

#Return compass angle as degrees
def getAngle(compass):
    compassVal = compass.getValues()
    if not math.isnan(compassVal[0]):
        return math.degrees(math.atan2(compassVal[0], compassVal[2]))
    else:
        return 'NULL'

#Get N,S,E,W for an x turn
#Turn is 1=right, -1=left, 2=backwards
def getTurnDirection(current, turn):
    compass = ['N', 'E', 'S', 'W']
    return compass[(compass.index(current)+turn)%4]

#Get compass value as N,S,E,W
def getComVal(c):
    angle = getAngle(c)
    if not angle == 'NULL':
        if angle >= 45 and angle < 135:
            return('N')
        elif angle >= 135 or angle <= -135:
            return('E')
        elif angle >= -135 and angle < -45:
            return('S')
        else:
            return('W')

#Loop to turn in x direction
def turn(rob, c, l, r, direction):
    angle = getAngle(c)
    targetComp = getTurnDirection(getComVal(c), direction)
    while rob.step(TIME_STEP) != -1 and not isStraight(angle, targetComp):
        angle = getAngle(c)
        if not isStraight(angle, targetComp):
            l.setVelocity(MAX_SPEED)
            r.setVelocity(-1 * MAX_SPEED)

opt = Optimizer()
write = not opt.doesFileExist()

# create the Robot instance.
robot = Robot()

com = robot.getCompass('compass')
com.enable(TIME_STEP)

acc = robot.getAccelerometer('accelerometer')
acc.enable(TIME_STEP)

currentPos = [0, 0]

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    # detect obstacles
    right_obstacle = psValues[0] > DETECT_VAL or psValues[1] > DETECT_VAL or psValues[2] > DETECT_VAL
    left_obstacle = psValues[5] > DETECT_VAL or psValues[6] > DETECT_VAL or psValues[7] > DETECT_VAL

    front_obstacle = psValues[7] > DETECT_VAL and psValues[0] > DETECT_VAL

    right_path = psValues[2] < 63.0
    left_path = psValues[5] < 63.0

    #Calculate current position
    #SENSITIVE TO GETTING STUCK, LEADS TO DRIFT
    print(write)
    if write:
        delta = acc.getValues()
        delta = [delta[0], delta[2]]
        angle = getAngle(com)
        if not angle == "NULL":
            delta = rotate(delta, angle)
            currentPos[0] += (delta[0]/10)
            currentPos[1] += (delta[1]/10)
            #print(currentPos)
            opt.writeData(currentPos)

    if front_obstacle:
        turn(robot, com, leftMotor, rightMotor, 2)

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  += 0.05 * MAX_SPEED
        rightSpeed -= 0.05 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  -= 0.05 * MAX_SPEED
        rightSpeed += 0.05 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
