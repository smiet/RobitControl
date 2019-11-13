import serial
import math
import pygame
import time
#import matplotlib.pyplot as plt
#from PID import PID

#NOTES  Need to fix the phi calculation
#       Need to fix the distance between leg tip and leg root based off tilt angle


#Initalise some global variables: NEW MOTOR CHANGE
#s = serial.Serial(port= '/dev/ttyUSB0', baudrate = 115200)
s = serial.Serial(port= 'COM3', baudrate = 38400)
# y = [0. for x in range(3)]
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

bounds = 0.02   #How far the robot will move before changing legs
#Add hook for center of mass checks
upleg = 0    #Counter variable used to keep track of which legs are up and down
xposinit = 0    #The x position of the robot
yposinit = 0    #The y position of the robot
xposleg = [0 for x in range(0,4)] #Set this to the actual INITIAL values of each leg. Robot should start with 3 legs down, 1 up all equidistant.
yposleg = [0 for x in range(0,4)]


theta[0][0] = 0 # more positive is anticlockwise
theta[0][1] = 0 # more negative is higher leg
theta[0][2] = 0 # more negative is higher leg
theta[1][0] = 0
theta[1][1] = 0
theta[1][2] = 0
theta[2][0] = 0
theta[2][1] = 0
theta[2][2] = 0
theta[3][0] = 0
theta[3][1] = 0
theta[3][2] = 0

# Initialise the legs by putting them all up.
def ssc_init():
    global zmidps
    inittheta = [[0. for x in range(3)] for x in range(4)]
    for i in range(0,4):
        inittheta[i][1] = 00*math.pi/180 # knee
        inittheta[i][2] = 00*math.pi/180 # ankle
    theta = offsettheta(inittheta)
    tossc(theta)
    
    #Find current r value to determine xinit and yinit. Assuming phi = 0
    for i in range (0,4):
        rth = l1*math.cos(inittheta[i][1]) + l2*math.cos(inittheta[i][1] + inittheta[i][2])
        xinit[i] = math.cos(-legdiff[i])*(r + l0 + rth)
        yinit[i] = math.sin(-legdiff[i])*(r + l0 + rth)
    print(xinit[2], yinit[2])
    #Determine height of the middle of the upper platform so it can be used in psmode() as a midpoint for the height
    zmidps = h + l1*math.sin(inittheta[i][1]) + l2*math.sin(inittheta[i][1] + inittheta[i][2])
    print(zmidps)
    print ("AMPUTATED TIPSY INITIALISED")

    
j = 0
rev=0
Zmid = 5/100



def legpostotheta(legpos):
    L1 = math.sqrt(legpos[0]*legpos[0]+legpos[1]*legpos[1])
    L = math.sqrt(legpos[2]*legpos[2]+(L1-l0)**2)
    gamma = math.atan2(legpos[1],legpos[0])
    alpha = math.acos(legpos[2]/L) + math.acos((l2*l2-l1*l1-L*L)/(-2*l1*L))
    beta = math.acos((L*L-l2*l2-l1*l1)/(-2*l2*l1))
    return (gamma,alpha,beta)

while 1:
    zheight=j/100;
    theta = plattotheta(0,0,Zmid,xpos,ypos)
    tossc(theta)
    
    s.write("<#1 P500 #2 P400 #3 P%s #4 P300 #5 P600 #6 P600 #7 P500 #8 P600 #9 P600 #10 P500 #11 P400 #12 P400k>" % (600+j*2))
    #Write the height here instead
    if j == 50:
        rev = 1
        print("REVERSING NEXT")
    if j == -50:
        rev = 0
        print("REVERSING NEXT")
    if rev == 0:
        j = j+1
    else:
        j = j-1
    time.sleep(0.035)    




#This function takes in a wanted thetax, thetay, and Zmid and sends the actual signal to the ssc-32u (servocontroller)
def tossc(theta):
    #Send the new servo angles obtained by the IK Engine to the servo controller:
    s.write("<#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk>" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))
    #print("#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))
    #print("<#1 P%s #2 P%s #3 P%s #4 P%s #5 P%s #6 P%s #7 P%s #8 P%s #9 P%s #10 P%s #11 P%s #12 P%sk>" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2])))
    #s.write("<#1 P500 #2 P500 #3 P500 #4 P500 #5 P500 #6 P500 #7 P500 #8 P500 #9 P500 #10 P500 #11 P500 #12 P500k>"
    return


#Angle Conversion formula NEW MOTOR CHANGE. AX-12 Go from 0 to 1023 so 1024 positions. This is between -150 to + 150 deg.
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


#This function will take the theta from the model and spit out the values the robot needs based off its physical imperfections NEW MOTOR CHANGE
def offsettheta(oldtheta):
    newtheta = [[0. for x in range(3)] for x in range(4)]
    thetaoffset = [[0. for x in range(3)] for x in range(4)]
    #Leg Set 1. Inital legs on ground.
    thetaoffset[0][0] = -0.*math.pi/180 # more positive is anticlockwise
    thetaoffset[0][1] = +0.*math.pi/180 # more negative is higher leg
    thetaoffset[0][2] = -0*math.pi/180 # more negative is higher leg
    thetaoffset[1][0] = -0*math.pi/180.
    thetaoffset[1][1] = +0*math.pi/180
    thetaoffset[1][2] = +0.0*math.pi/180
    thetaoffset[2][0] = +0.0*math.pi/180  #
    thetaoffset[2][1] = +0.*math.pi/180
    thetaoffset[2][2] = +0.0*math.pi/180
    thetaoffset[3][0] = 0.*math.pi/180   #
    thetaoffset[3][1] = +0*math.pi/180
    thetaoffset[3][2] = -0*math.pi/180

    newtheta[0][0] = oldtheta[0][0] + thetaoffset[0][0] #1
    newtheta[0][1] = oldtheta[0][1] + thetaoffset[0][1] #2
    newtheta[0][2] = -oldtheta[0][2] + thetaoffset[0][2] #3
    newtheta[1][0] = oldtheta[1][0] + thetaoffset[1][0] #4
    newtheta[1][1] = -oldtheta[1][1] + thetaoffset[1][1] #5
    newtheta[1][2] = oldtheta[1][2] + thetaoffset[1][2] #6
    newtheta[2][0] = oldtheta[2][0] + thetaoffset[2][0] #7
    newtheta[2][1] = -oldtheta[2][1] + thetaoffset[2][1] #8
    newtheta[2][2] = oldtheta[2][2] + thetaoffset[2][2] #9
    newtheta[3][0] = oldtheta[3][0] + thetaoffset[3][0] #10
    newtheta[3][1] = oldtheta[3][1] + thetaoffset[3][1] #11
    newtheta[3][2] = -oldtheta[3][2] + thetaoffset[3][2] #12


    return newtheta



# This function takes in thetax, thetay, and Zmid values and determines which servo angles are needed, which are returned as theta - a 2d array
# The math behind it was done in mathematica file 'Theta Angles Working.nb'
def plattotheta(thetax, thetay, Zmid,xpostemp, ypostemp):
    
    #Initialise some variables:
    zup = [0. for x in range(4)]
    xpos = [0. for x in range(4)]
    ypos = [0. for x in range(4)]
    thetamodel = [[0. for x in range(3)] for x in range(4)]

    """
    if upleg ==0:
        thetax = thetax - 0.2*math.pi/180
        thetay = thetay - 0.2*math.pi/180
    else:
        thetax = thetax - 1*math.pi/180
        thetay = thetay + 1*math.pi/180
    """
    #Find the wanted positions for the grounded legs
    """
    for i in [j for j in xrange(0,4) if j != upleg]:
        zup[i] = 0
        xpos[i] = xpostemp - xposinit + xinit[i]
        ypos[i] = ypostemp - yposinit + yinit[i]
    """    
    for i in [j for j in xrange(0,4)]:
        zup[i] = 0
        xpos[i] = xpostemp - xposinit + xinit[i]
        ypos[i] = ypostemp - yposinit + yinit[i]

    #Determine how high the up legs should be
    zupmax = -0.07
    #rinit = math.sqrt(xinit[i]**2. + yinit[i]**2.)
    #rpos = math.sqrt(xpos[i]**2. + ypos[i]**2.)
    reach = math.sqrt((xpostemp-xposinit)**2 + (ypostemp - yposinit)**2)
    
    zuptemp = zupmax/bounds*(0 + bounds - reach) # Z position of the up leg
    #Find the wanted positions for the up leg
    """
    zup[upleg] = zuptemp
    xpos[upleg] = xinit[upleg]
    ypos[upleg] = xinit[upleg]
    """
    #print(xpos)
    #print(ypos)
    
    #Go through each leg and determine the thetas each motor should have
    for i in range(0,4):
        zp = r*math.cos(-legdiff[i])*math.sin(thetax)+r*math.sin(-legdiff[i])*math.sin(thetay)
        xp = r*math.cos(-legdiff[i])*math.cos(thetax)
        yp = r*math.sin(-legdiff[i])*math.cos(thetay)
        
        zl = Zmid - zp - zup[i]
        xl = xpos[i] - xp
        yl = ypos[i] - yp

        inatanx = -(xl*math.cos(thetax)+zl*math.sin(thetax))/(xl*xl*math.cos(thetax)**2.+yl*yl*math.cos(thetay)**2.+zl*(zl*math.sin(thetax)**2.+xl*math.sin(2*thetax)+zl*math.sin(thetay)**2.+yl*math.sin(2*thetay)))
        inatany = (-yl*math.cos(thetay)-zl*math.sin(thetay))/(xl*xl*math.cos(thetax)**2.+yl*yl*math.cos(thetay)**2.+zl*(zl*math.sin(thetax)**2.+xl*math.sin(2*thetax)+zl*math.sin(thetay)**2.+yl*math.sin(2*thetay)))
        #theta0=0
        theta0 = math.atan2(inatany,inatanx)

        rl = abs(xl*math.cos(theta0)*math.cos(thetax)+yl*math.cos(thetay)*math.sin(theta0)+zl*(math.cos(theta0)*math.sin(thetax)+math.sin(theta0)*math.sin(thetay)))
        shouldbezero = yl*math.cos(theta0)*math.cos(thetay)-xl*math.cos(thetax)*math.sin(theta0)+zl*(-math.sin(theta0)*math.sin(thetax)+math.cos(theta0)*math.sin(thetay)) #Could probably comment this out to save on computation time
        zlnew = zl*math.cos(thetax)*math.cos(thetay)-xl*math.cos(thetay)*math.sin(thetax)-yl*math.cos(thetax)*math.sin(thetay)
        zl = abs(zlnew)
        #print(rl, i)
        #print(shouldbezero,i)
        #print(zl, i)
        thetamodel[i][0] = theta0 + legdiff[i]
        
        if thetamodel[i][0] > 2:
            thetamodel[i][0] = thetamodel[i][0] - math.pi
        if thetamodel[i][0] < -2:
            thetamodel[i][0] = thetamodel[i][0] + math.pi
        
        #print(thetamodel[i][0],i)

        #phi = math.atan2(-(math.sin(thetamodel[i][0] - legdiff[i])*math.cos(thetax)*math.sin(thetay) + math.cos(thetamodel[i][0] - legdiff[i])*math.sin(thetax)*math.cos(thetay)),math.cos(thetax)*math.cos(thetay))
        phi = 0
        zoth = l0*math.sin(phi) + h*math.cos(phi)
        roth = l0*math.cos(phi) - h*math.sin(phi)
        zth = zl - zoth
        rth = rl - roth
        #print("rth:", rth)
        #print("zth:", zth)
        #Begin the math that finds theta1:
        inroot = - l1**4. - (- l2**2. + rth**2. + zth**2.)**2. + 2*l1**2. *(l2**2. + rth**2. +zth**2.)
        if inroot < 0:
            inroot = 0
            print("the inroot of leg,", i, "is below 0")
        inacos = (rth*(l1*l1 - l2*l2 + rth*rth + zth*zth) + zth*math.sqrt(inroot))/(2.*l1*(rth**2. + zth**2.))
        if inacos > 1:
            inacos = 1.
            print("the inacos of leg,", i, "is above 1")
        if inacos < -1:
            inacos = -1.
            print("the inacos of leg,", i, "is below -1")
        
        phi1 =-math.acos(inacos)
        thetamodel[i][1] = phi1 - phi
        thetamodel[i][2] = math.acos((rth - l1*math.cos(phi1))/l2)-phi1
        """
        rth = l1*math.cos(thetamodel[i][1] + phi) + l2*math.cos(thetamodel[i][1] + thetamodel[i][2] + phi)
        roth = l0*math.cos(phi) - h*math.sin(phi)
        rlegtest = rth + roth
        xltest = math.cos(-legdiff[i] + thetamodel[i][0])*rlegtest
        yltest = math.sin(-legdiff[i] + thetamodel[i][0])*rlegtest
        xtest = xltest + r*math.cos(-legdiff[i])*math.cos(thetax)
        ytest = yltest + r*math.sin(-legdiff[i])*math.cos(thetay)
        zth = l1*math.sin(thetamodel[i][1] + phi) + l2*math.sin(thetamodel[i][1] + thetamodel[i][2] + phi)

        zltest = zth + zoth
        ztest = zltest + r*math.cos(-legdiff[i])*math.sin(thetax) + r*math.sin(-legdiff[i])*math.sin(thetay)
        xdifftest = xtest - xinit[i]
        ydifftest = ytest - yinit[i]
        zdifftest = ztest - Zmid + zup[i]
        if abs(xdifftest) < 0.0001:
            xdifftest = 0
        if abs(ydifftest) < 0.0001:
            ydifftest = 0
        if abs(zdifftest) < 0.0001:
            zdifftest = 0
        print(i, "x difference:", xdifftest, "y difference:", ydifftest, "z difference:", zdifftest )
        """
        
        thetamodel[upleg][1] = -40*math.pi/180
        thetamodel[upleg][2] = -40*math.pi/180
    #Add the thetaoffset to the final value
    theta = offsettheta(thetamodel)

    return theta





#Emergency function to put the motors in a default position.
    """
def NO():
    s.write("#0 P1500 #1 P1500 #2 P1500 #4 P1500 #5 P1500 #6 P1500 #8 P1500 #9 P1500 #10 P1500 #16 P1500 #17 P1500 #18 P1500 #20 P1500 #21 P1500 #22 P1500 #24 P1500 #25 P1500 #26 P1500 T100 \r")
    
def MAX():
    s.write("#0 P2400 #1 P2400 #2 P2400 #4 P2400 #5 P2400 #6 P2400 #8 P2400 #9 P2400 #10 P2400 #16 P2400 #17 P2400 #18 P2400 #20 P2400 #21 P2400 #22 P2400 #24 P2400 #25 P2400 #26 P2400 T100 \r")

def MIN():
    s.write("#0 P600 #1 P600 #2 P600 #4 P600 #5 P600 #6 P600 #8 P600 #9 P600 #10 P600 #16 P600 #17 P600 #18 P600 #20 P600 #21 P600 #22 P600 #24 P600 #25 P600 #26 P600 T100 \r")

"""
#Start playstation controller commands
def psmode():
    while 1:
        #-------------Set up the first variables
        global upleg, xposinit, yposinit, zoffset    #These variables will be changed globally
        triangle = 0
        #startbutton = 0
        circle = 0
        square = 0
        xbutton = 0
        xpos = xposinit
        ypos = yposinit
        thetax = 0
        thetay = 0
        Zmid = zmidps
        Ki = 0
        Kp = 0
        Kd = 0
        uneven = 0
        position = [0 for x in range(2)]
        print(position)
        #arduino = serial.Serial('/dev/ttyACM0', 115200)
        """
        px = PID(Kp,Ki,Kd)
        px.SetPoint = 0.0
        py = PID(Kp,Ki,Kd)
        py.SetPoint = 0.0
        px.setSampleTime(0.03)
        py.setSampleTime(0.03)
        px.setWindup(0.5)
        py.setWindup(0.5)
        """
        
        pygame.init()
        print("PS mode activated")
        #Set leg angles initially
        theta = plattotheta(thetax,thetay,Zmid,xpos,ypos)
        tossc(theta)
        print("PLEASE DELAY A BIT")
        time.sleep(1)
        joystick_count = pygame.joystick.get_count()
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
        startbutton = joystick.get_button(3)
        pygame.event.pump()
        uneventimer = 0

        #Manual movement mode (square is pressed)
        while (circle == 0) and (startbutton ==0):
            #print("tilt mode activated")
            pygame.event.pump()

            #----------------Height calculation
            #Zmid = zmidps*(-joystick.get_axis(12)*0.2+1)
            #Zmid = zmidps*((-joystick.get_axis(2))/3+1)
            Zmid = zmidps
            #print((-joystick.get_axis(12) + joystick.get_axis(13)+2)/2)

            #----------------Thetax and Thetay input
            thetax = 0
            thetay = 0
            #thetax = joystick.get_axis(4)*0.3
            #thetay = -joystick.get_axis(3)*0.3

            #---------------Walking algorithms
            xdirection = 0
            ydirection = 0
            if abs(joystick.get_axis(1)) < 0.1:
                ydirection = 0
            else:
                ydirection = joystick.get_axis(1)*0.002
            if abs(joystick.get_axis(0)) < 0.1:
                xdirection = 0
            else:
                xdirection = joystick.get_axis(0)*0.002
            #print(joystick.get_axis(1))
            #xdirection = -joystick.get_axis(0)*0.002
            #ydirection = joystick.get_axis(1)*0.002
            xpos,ypos = walking(xdirection, xpos, ydirection, ypos,uneven)
            

            
            #----------------Send everything to the servo controller
            theta = plattotheta(thetax,thetay,Zmid,xpos,ypos)
            tossc(theta)
            

            
            
            circle = joystick.get_button(1)
            startbutton = joystick.get_button(7)
            xbutton = joystick.get_button(0)
            if (xbutton == 1):
                if (upleg == 3):
                    upleg = 0
                    print("upleg is now", upleg)
                    time.sleep(0.5)
                else:
                    upleg = upleg + 1
                    print("upleg is now", upleg)
                    time.sleep(0.5)
            square = joystick.get_button(2)
        square = 0

#def area(pos1,x2,y2,x3,y3):
#    return abs((x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2.0
               
#This function determines which leg should be moved and provides the new positions for the robot as it moves

def walking(xdirection, xpos, ydirection, ypos, uneven):
    global xposinit, yposinit, upleg
    xposnew = xpos + xdirection
    yposnew = ypos + ydirection
    #print(xposnew,yposnew)
    for i in range(0,4):
        reach = (xposnew-xposinit)**2 + (yposnew - yposinit)**2
        if (reach > bounds*bounds):
            xposnew = xpos
            yposnew = ypos
            print("Hit Bounds")
            print(xposnew,yposnew)
            print(reach)

                
            
    #time.sleep(0.1)    
    return xposnew, yposnew
"""
def walking(xdirection, xpos, ydirection, ypos, uneven):
    global xposinit, yposinit, upleg
    xpos = xpos + xdirection
    ypos = ypos + ydirection
    if uneven == 0:
        reach = [0 for x in range(0,4)]
        masscenter = [0 for x in range(0,4)]
        #Determine if any of the legs are moving too far, and if so, change legs by determining which leg is optimal to change
        for i in range(0,4):
            reach[i]=(xpos+r*math.cos(legdiff[i])-xposleg[i])**2 + (ypos+r*math.sin(legdiff[i]) - yposleg[i])**2
        #Find center of mass for all leg combinations. Whichever is the closest to bounds (smallest sub-area) is candidate for leg switching. Perhaps a formula with a combination of that small area and closeness to leg limit and DIRECTION of motion should determine which leg switches
        #for i in range(0,4):
        #    Totalarea=area(xposleg[0],yposleg[0],xposleg[1],yposleg[1],xposleg[2],yposleg[2])
            
            #masscentercheck[i]=
        #Determine which leg to change to based on bound limits and center of mass
        #Determine if the bot is moving too far, and if so, change legs to continue walking
        if (reach > bounds*bounds):
            xposinit = xpos
            yposinit = ypos
            #SWITCH UP AND DOWN LEGS
            if (upleg == 0):
                upleg = 1
                zoffset[0] = 0
                zoffset[1] = 0
                zoffset[2] = 0
            else:
                upleg = 0
                zoffset[3] = 0
            print("SWITCHING THE LEGS BY WALKING")
            time.sleep(0.1)
    

    return xpos, ypos
"""

#Algorithm that allows the spider to walk on uneven terrain. It does this by triggering when an up foot sensor
#has a value of 1 and lowering the other feet until they also trigger. It then takes in the values of the z's
#and applies them on top of the needed z in the next step when those feet are on the ground.


time.sleep(5)        
ssc_init()
psmode()
