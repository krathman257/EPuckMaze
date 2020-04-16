from controller import Compass, Robot, DistanceSensor, Motor
import math
import sys
import re

class Optimizer:
    def __init__(self):
        self.counter = -1
        self.filePath = "./data.txt"
        self.file = self.read_in()
        self.data = []
        if self.doesFileExist():
            temp = self.file.readlines()

            #Read in each line as a list of floats
            for l in temp:
                d = l.strip('\n').split(',')
                for i in range(0, len(d)):
                    d[i] = float(d[i])
                self.data.append(d)

            #Check each point (from start) against
            #each point (from end) for loops
            for i in range(0, len(self.data)-1):
                for j in range(len(self.data)-1, i+5, -1):
                    if not self.checkPoints(i, j):
                        i = 0
                        j = len(self.data)-1
            print('File Opened and Processed')

    #Open file
    def read_in(self):
        try:
            file = open(self.filePath, "r")
            pass
        except:
            file = 0
        return file

    #Get next motor values, iterate
    def getNextMotorValues(self):
        self.counter += 1
        if self.counter == len(self.data):
            return False
        return [float(self.data[self.counter][2]), float(self.data[self.counter][3])]

    #Check if there's a file
    def doesFileExist(self):
        return not self.file == 0

    #See if distance between two points is acceptable
    #If not, delete detected loop and restart search
    def checkPoints(self, start, end):
        p1 = self.data[start][:2]
        p2 = self.data[end][:2]
        x = p1[0] - p2[0]
        y = p1[1] - p2[1]
        dist = math.sqrt((x * x) - (y * y))
        if dist < 3:
            self.data = self.data[:start]+self.data[end:]
            return False
        else:
            return True

    #Append a line of data to the file
    def writeData(self, pos, l, r):
        with open(self.filePath, "a") as f:
            f.write("%.4f,%.4f,%.3f,%.3f\n" % (pos[0], pos[1], l, r))
            f.close()

#Time in [ms] of a simulation step
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

#Create Optimizer instance
opt = Optimizer()
write = not opt.doesFileExist()
#print(opt.data)

#Create the Robot instance
robot = Robot()

com = robot.getCompass('compass')
com.enable(TIME_STEP)

acc = robot.getAccelerometer('accelerometer')
acc.enable(TIME_STEP)

currentPos = [0, 0]

#Initialize devices
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

#Feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    #Read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    #Detect obstacles
    right_obstacle = psValues[0] > DETECT_VAL or psValues[1] > DETECT_VAL or psValues[2] > DETECT_VAL
    left_obstacle = psValues[5] > DETECT_VAL or psValues[6] > DETECT_VAL or psValues[7] > DETECT_VAL
    front_obstacle = psValues[7] > DETECT_VAL and psValues[0] > DETECT_VAL

    right_path = psValues[2] < 63.0
    left_path = psValues[5] < 63.0

    if front_obstacle:
        turn(robot, com, leftMotor, rightMotor, 2)

    #Initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    #Modify speeds according to obstacles
    if left_obstacle:
        
        #Turn right
        leftSpeed  += 0.05 * MAX_SPEED
        rightSpeed -= 0.05 * MAX_SPEED
    elif right_obstacle:
        
        #Turn left
        leftSpeed  -= 0.05 * MAX_SPEED
        rightSpeed += 0.05 * MAX_SPEED

    #Calculate current position
    #SENSITIVE TO GETTING STUCK, LEADS TO DRIFT
    if write:
        delta = acc.getValues()
        delta = [delta[0], delta[2]]
        angle = getAngle(com)
        if not angle == "NULL":
            delta = rotate(delta, angle)
            currentPos[0] += (delta[0]/10)
            currentPos[1] += (delta[1]/10)
            opt.writeData(currentPos, leftSpeed, rightSpeed)

    #Write actuators inputs
    #If writing to file,
    if write:
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    #If reading from file,
    else:
        motorValues = opt.getNextMotorValues()
        if motorValues == False:
            sys.exit("Ran out of motor values")
        leftMotor.setVelocity(motorValues[0])
        rightMotor.setVelocity(motorValues[1])
