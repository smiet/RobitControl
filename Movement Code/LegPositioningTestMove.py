import serial
import math
import pygame
import time
import numpy as np

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
    thetaoffset[0][0] = -90.*math.pi/180 # more positive is anticlockwise
    thetaoffset[0][1] = -90.*math.pi/180 # more negative is higher leg
    thetaoffset[0][2] = 90*math.pi/180 # more negative is higher leg
    thetaoffset[1][0] = -90*math.pi/180.
    thetaoffset[1][1] = -90*math.pi/180
    thetaoffset[1][2] = +90.0*math.pi/180
    thetaoffset[2][0] = -90.0*math.pi/180  #
    thetaoffset[2][1] = -90.*math.pi/180
    thetaoffset[2][2] = +90.0*math.pi/180
    thetaoffset[3][0] = -90.*math.pi/180   #
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

"""
while 1:
    tossc(theta)
    time.sleep(1)
    print("waiting")
"""


j = 0
rev=0
initpos=0.16
def bodypostolegpos(bodypos):
    legpos[0] = [initpos + bodypos[0], bodypos[1], bodypos[2]]
    legpos[1] = [initpos + bodypos[1], -bodypos[0], bodypos[2]]
    legpos[2] = [initpos - bodypos[1], bodypos[0], bodypos[2]]
    legpos[3] = [initpos - bodypos[0], -bodypos[1], bodypos[2]]
    return legpos

def area(x1,x2,y2,x3,y3):
    return abs((x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2.0)

def masscentercheck(legposstep,bodypos,i):
    massindexes = [0,1,2,3]
    massindexes.remove(i) #This creates the index to check the center of mass for the other legs
    #Check if each combination of legs gives a triangle that the center of mass resides inside
    #To do so check if bodypos point is inside the points created by legs 0-2
    temppt0 = [legposstep[massindexes[0]][0], legposstep[massindexes[0]][1]]
    temppt1 = [legposstep[massindexes[1]][0], legposstep[massindexes[1]][1]]
    temppt2 = [legposstep[massindexes[2]][0], legposstep[massindexes[2]][1]]
    tempbody = bodypos
    test = ptInTriang(bodypos,temppt0,temppt1,temppt2)
    #print(test)
    #print(legposstep[0], legposstep[1], legposstep[2], bodypos)
    return test

def ptInTriang(p_test, p0, p1, p2):       
     dX = p_test[0] - p0[0]
     dY = p_test[1] - p0[1]
     dX20 = p2[0] - p0[0]
     dY20 = p2[1] - p0[1]
     dX10 = p1[0] - p0[0]
     dY10 = p1[1] - p0[1]

     s_p = (dY20*dX) - (dX20*dY)
     t_p = (dX10*dY) - (dY10*dX)
     D = (dX10*dY20) - (dY10*dX20)

     if D > 0:
         return (  (s_p >= 0) and (t_p >= 0) and (s_p + t_p) <= D  )
     else:
         return (  (s_p <= 0) and (t_p <= 0) and (s_p + t_p) >= D  )
       

#As it walks, bodypos is changing.
#bodypos - bodyposinit is the variable which it uses for movement,
#which keeps track of the last place it stepped (bodyposinit)
#Calculate the leg position change by (bodypos-bodyposinit)
#When leg steps, that particular leg changes its legposinit which it uses for that relative movement calc by bodypos
#Actual position of the leg relative to body should then be (bodypos - legpos)
#New position vector for leg frame and angle calcs - legposLF (leg position leg frame)

def legposTOlegposLF(legposstep,bodypos):
    legpos = [[0. for x in range(3)] for x in range(4)]
    legpos[0] = [legposstep[0][0]-bodypos[0], legposstep[0][1]-bodypos[1]-r, Zmid]
    legpos[1] = [legposstep[1][1]-bodypos[1], -(legposstep[1][0]-bodypos[0])-r, Zmid]
    legpos[2] = [-(legposstep[2][1]-bodypos[1]), legposstep[2][0]-bodypos[0]-r, Zmid]
    legpos[3] = [-(legposstep[3][0]-bodypos[0]), -(legposstep[3][1]-bodypos[1])-r, Zmid]
    return legpos

def walking(boop):
    legpos = [[0. for x in range(3)] for x in range(4)]
    legposLF = [[0. for x in range(3)] for x in range(4)]
    legposstep = [[0. for x in range(3)] for x in range(4)]
    Zmid = 0.12
    
    bodyposinit = [0.05, 0.0, Zmid]
    bodypos = bodyposinit
    legposstep[0] = [0, initpos+r, Zmid]
    legposstep[1] = [-initpos-r, 0, Zmid]
    legposstep[2] = [initpos+r, 0, Zmid]
    legposstep[3] = [0, -initpos-r, Zmid]
    legposstepINIT = legposstep #Save this variable for now....temporary
    j = 0
    rev = 0
    upleg = 0 #This is the leg that is initially up
    Zup = 0.02 #The amount the upleg is raised from the floor
    tempmove = 1
    from operator import add
    while 1:
        delbody = [0,float(-j*tempmove)/500,0.0] #Test code for the direction the robot is given for movement
        bodypos = np.asarray(bodypos) - np.asarray(delbody) #Move the robot body by the directed amount
        tempmove = 1
        #Alter upleg motion
        #Move upleg in the direction of motion
        legposstep[upleg] = [legposstep[upleg][0], legposstep[upleg][1], Zmid-Zup] #Make sure leg is slightly raised
        legposstep[upleg] = np.asarray(legposstep[upleg]) - 4*np.asarray(delbody) #Hackey for now
        #print(legposstep[upleg],bodypos)
        legposLF = legposTOlegposLF(legposstep,bodypos) #Determine the leg position vector in relation to body frame
        


        
        #Search through the legs and determine if any legs provide a combination that would give an unbalanced robot
        legcheck = masscentercheck(legposstep,bodypos,upleg)
        if legcheck == False:
            #If about to be unstable, check which combination will give stability
            for i in range(4):
                legcheckscan = masscentercheck(legposstep,bodypos,i)
                if legcheckscan == True:
                    upleg = i #Change the upleg with the one to provide that found stability.
                    legposstep = legposstepINIT #Temporary push to make legs go back to normal for now
                    print("Changing upleg to ", upleg)
        

                    
        """            
        for i in range(4):
            legcheck = masscentercheck(legposstep,bodypos,i)
            if legcheck == False:
                print("This leg cannot go up", i)
        """
        #templegpos = bodypostolegpos(tempbodypos)
        temptheta = [[0. for x in range(3)] for x in range(4)]
        temptheta[0]=legpostotheta(legposLF[0])
        temptheta[1]=legpostotheta(legposLF[1])
        temptheta[2]=legpostotheta(legposLF[2])
        temptheta[3]=legpostotheta(legposLF[3])
        theta = offsettheta(temptheta)
        tossc(theta)
        #print("<#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk>" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))

        botmovement = np.asarray(bodypos) - np.asarray(bodyposinit)
        print(botmovement)
        if botmovement[1] > 0.0533:
            rev = 1
            tempmove = 5
            print("REVERSING NEXT")
            print("Bot position", botmovement)
            #upleg = 2
        if botmovement[1] < -0.0533:
            rev = 0
            tempmove = 5
            print("REVERSING NEXT")
            print("Bot position", botmovement)
            #upleg = 1
        if rev == 0:
            j = 1
        else:
            j = -1
        time.sleep(0.04)
Zmid = 0.12
walking(1)
    
