import serial
import math
import pygame
import time

xinit = [0. for x in range(4)]
yinit = [0. for x in range(4)]
zoffset = [0. for x in range(4)]

#leg angle differences - used for theta0 calculations.
legdiff = [0 for x in range(0,4)]
legdiff[0] = 0*math.pi/180
legdiff[1] = 90*math.pi/180
legdiff[2] = -90*math.pi/180
legdiff[3] = -180*math.pi/180

#Set values for the robot dimensions. Must be tweaked to match actual robot NEW MOTOR CHANGE
h = 0.0508   #Height distance between motor 1 and top of the platform
#h = 0.0
l0 = 0.0508    #Distance from motor 0 to motor 1
l1 = 0.079375    #Distance from motor 1 to motor 2
l2 = 0.14605     #Distance from motor 2 to leg tip
r = 0.0904875     #Distance from center of platform to motor 0

s = serial.Serial(port= 'COM3', baudrate = 38400)


def legpostotheta(legpos):
    L1 = math.sqrt(legpos[0]*legpos[0]+legpos[1]*legpos[1])
    L = math.sqrt(legpos[2]*legpos[2]+(L1-l0)**2)
    gamma = math.atan2(legpos[1],legpos[0])
    alpha = math.acos(legpos[2]/L) + math.acos((l2*l2-l1*l1-L*L)/(-2*l1*L))
    beta = math.acos((L*L-l2*l2-l1*l1)/(-2*l2*l1))
    return (gamma,alpha,beta)


def thetatoservo(angle):
    #sscnumber = angle*1024/300*180/math.pi* + 512. #Assuming in radians
    sscnumber = angle*195.56959 + 512. #Assuming in radians    
    if sscnumber > 800:
        sscnumber = 800
        #print("YOU'RE TOO HIGH")
    if sscnumber < 200:
        sscnumber = 200
        #print("YOU'RE TOO LOW")
    sscnumberout = int(round(sscnumber))
    return sscnumberout

def offsettheta(oldtheta):
    newtheta = [[0. for x in range(3)] for x in range(4)]
    thetaoffset = [[0. for x in range(3)] for x in range(4)]
    #Leg Set 1. Inital legs on ground.
    thetaoffset[0][0] = -0.*math.pi/180 # more positive is anticlockwise
    thetaoffset[0][1] = -90.*math.pi/180 # more negative is higher leg
    thetaoffset[0][2] = 90*math.pi/180 # more negative is higher leg
    thetaoffset[1][0] = -0*math.pi/180.
    thetaoffset[1][1] = -90*math.pi/180
    thetaoffset[1][2] = +90.0*math.pi/180
    thetaoffset[2][0] = +0.0*math.pi/180  #
    thetaoffset[2][1] = -90.*math.pi/180
    thetaoffset[2][2] = +90.0*math.pi/180
    thetaoffset[3][0] = 0.*math.pi/180   #
    thetaoffset[3][1] = -90*math.pi/180
    thetaoffset[3][2] = 90*math.pi/180

    newtheta[0][0] = (oldtheta[0][0] + thetaoffset[0][0]) #1
    newtheta[0][1] = -(oldtheta[0][1] + thetaoffset[0][1]) #2
    newtheta[0][2] = -(-oldtheta[0][2] + thetaoffset[0][2]) #3
    newtheta[1][0] = (oldtheta[1][0] + thetaoffset[1][0]) #4
    newtheta[1][1] = (oldtheta[1][1] + thetaoffset[1][1]) #5
    newtheta[1][2] = (-oldtheta[1][2] + thetaoffset[1][2]) #6
    newtheta[2][0] = oldtheta[2][0] + thetaoffset[2][0] #7
    newtheta[2][1] = (oldtheta[2][1] + thetaoffset[2][1]) #8
    newtheta[2][2] = (-oldtheta[2][2] + thetaoffset[2][2]) #9
    newtheta[3][0] = (oldtheta[3][0] + thetaoffset[3][0]) #10
    newtheta[3][1] = -(oldtheta[3][1] + thetaoffset[3][1]) #11
    newtheta[3][2] = -(-oldtheta[3][2] + thetaoffset[3][2]) #12


    return newtheta


#This function takes in a wanted thetax, thetay, and Zmid and sends the actual signal to the ssc-32u (servocontroller)
def tossc(theta):
    #Send the new servo angles obtained by the IK Engine to the servo controller:
    s.write("<#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk>" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))
    #print("#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))
    #print("<#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk>" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))
    #s.write("<#1 P500 #2 P500 #3 P500 #4 P500 #5 P500 #6 P500 #7 P500 #8 P500 #9 P500 #10 P500 #11 P500 #12 P500k>"
    return
#0.27166 is the limit of the leg in the x and y with 5 cm z position. 19.2cm for 45 deg both legs
legpos0 = [0.1920/2, 0.1921/2, 0.06]
legpos1 = [0.1920/2, 0.1921/2, 0.12]
legpos2 = [0.1920/2, 0.1921/2, 0.12]
legpos3 = [0.1920/2, 0.1921/2, 0.12]
temptheta = [[0. for x in range(3)] for x in range(4)]
temptheta[0]=legpostotheta(legpos0)
temptheta[1]=legpostotheta(legpos1)
temptheta[2]=legpostotheta(legpos2)
temptheta[3]=legpostotheta(legpos3)
theta = offsettheta(temptheta)
testing = theta[1]
print(testing)
print("<#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk>" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))


while 1:
    tossc(theta)
    time.sleep(1)
    print("waiting")
