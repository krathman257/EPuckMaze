from controller import Compass, Robot, DistanceSensor, Motor
import math

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

DETECT_VAL = 100.0

#turn is 1=right, -1=left, 2=backwards
def getTurnDirection(current, turn):
    compass = ['N', 'E', 'S', 'W']
    return compass[(compass.index(current)+turn)%4]

def getComVal(c):
    compassVal = c.getValues()
    if not math.isnan(compassVal[0]):
        angle = math.degrees(math.atan2(compassVal[0], compassVal[2]))
        if angle >= 45 and angle < 135:
            return('N')
        elif angle >= 135 or angle <= -135:
            return('E')
        elif angle >= -135 and angle < -45:
            return('S')
        else:
            return('W')

def turn(rob, c, l, r, direction):
    currentComp = getComVal(c)
    targetComp = getTurnDirection(currentComp, direction)
    while rob.step(TIME_STEP) != -1 and not currentComp == targetComp:
        currentComp = getComVal(c)
        if not currentComp == targetComp:
            l.setVelocity(MAX_SPEED)
            r.setVelocity(-1 * MAX_SPEED)

# create the Robot instance.
robot = Robot()
com = robot.getCompass('compass')
com.enable(TIME_STEP)

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

    print("Left: ",left_path,", Right: ",right_path)

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
