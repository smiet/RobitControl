import serial
import math
import pygame
import time
import matplotlib.pyplot as plt
from PID import PID

#NOTES  Need to fix the phi calculation
#       Need to fix the distance between leg tip and leg root based off tilt angle


#Initalise some global variables:
s = serial.Serial(port= '/dev/ttyUSB0', baudrate = 115200)
arduino = serial.Serial('/dev/ttyACM0', 115200)
# y = [0. for x in range(3)]
xinit = [0. for x in range(6)]
yinit = [0. for x in range(6)]
zoffset = [0. for x in range(6)]

#leg angle differences - used for theta0 calculations.
legdiff = [0 for x in range(0,6)]
legdiff[0] = -150*math.pi/180
legdiff[1] = 90*math.pi/180
legdiff[2] = -30*math.pi/180
legdiff[3] = 30*math.pi/180
legdiff[4] = -90*math.pi/180
legdiff[5] = +150*math.pi/180

#Set values for the robot dimensions. Must be tweaked to match actual robot
h = 0.0784603   #Height distance between motor 1 and top of the platform
#h = 0.0
l0 = 0.0621226    #Distance from motor 0 to motor 1
l1 = 0.06555    #Distance from motor 1 to motor 2
l2 = 0.127     #Distance from motor 2 to leg tip
r = 0.0970582     #Distance from center of platform to motor 0

bounds = 0.05   #How far the robot will move before changing legs
whichleg = 1    #Counter variable used to keep track of which legs are up and down
xposinit = 0    #The x position of the robot
yposinit = 0    #The y position of the robot

#PID Control system variables
span = 10    #How many points the PID control system uses in integration and derivatives.
ballx = [0. for x in range(span)]
bally = [0. for x in range(span)]


def flushbuffer():
    while arduino.inWaiting() > 0:
        arduino.readline()

#This function takes in a wanted thetax, thetay, and Zmid and sends the actual signal to the ssc-32u (servocontroller)
def tossc(theta):
    #Send the new servo angles obtained by the IK Engine to the servo controller:
    s.write("#0 P%s #1 P%s #2 P%s #4 P%s #5 P%s #6 P%s #8 P%s #9 P%s #10 P%s #16 P%s #17 P%s #18 P%s #20 P%s #21 P%s #22 P%s #24 P%s #25 P%s #26 P%s T100 \r" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2]),thetatoservo(theta[4][0]),thetatoservo(theta[4][1]),thetatoservo(theta[4][2]),thetatoservo(theta[5][0]),thetatoservo(theta[5][1]),thetatoservo(theta[5][2])))
    #print ("#0 P%s #1 P%s #2 P%s #4 P%s #5 P%s #6 P%s #8 P%s #9 P%s #10 P%s #16 P%s #17 P%s #18 P%s #20 P%s #21 P%s #22 P%s #24 P%s #25 P%s #26 P%s T100 \r" % (thetatoservo(theta[0][0]),thetatoservo(theta[0][1]),thetatoservo(theta[0][2]),thetatoservo(theta[1][0]),thetatoservo(theta[1][1]),thetatoservo(theta[1][2]),thetatoservo(theta[2][0]),thetatoservo(theta[2][1]), thetatoservo(theta[2][2]),thetatoservo(theta[3][0]),thetatoservo(theta[3][1]),thetatoservo(theta[3][2]),thetatoservo(theta[4][0]),thetatoservo(theta[4][1]),thetatoservo(theta[4][2]),thetatoservo(theta[5][0]),thetatoservo(theta[5][1]),thetatoservo(theta[5][2])))
    return


#Angle Conversion formula
def thetatoservo(angle):
    sscnumber = angle*2000./math.pi + 1500. #Assuming in radians
    if sscnumber > 2400:
        sscnumber = 2400
        #print("YOU'RE TOO HIGH")
    if sscnumber < 530:
        sscnumber = 530
        #print("YOU'RE TOO LOW")
    sscnumberout = int(round(sscnumber))
    return sscnumberout


#This function will take the theta from the model and spit out the values the robot needs based off its physical imperfections
def offsettheta(oldtheta):
    newtheta = [[0. for x in range(3)] for x in range(6)]
    thetaoffset = [[0. for x in range(3)] for x in range(6)]
    #Leg Set 1. Inital legs on ground.
    thetaoffset[0][0] = -36.*math.pi/180 # more positive is anticlockwise
    thetaoffset[0][1] = +12.*math.pi/180 # more negative is higher leg
    thetaoffset[0][2] = -58*math.pi/180 # more negative is higher leg
    thetaoffset[1][0] = -9*math.pi/180.
    thetaoffset[1][1] = +26*math.pi/180
    thetaoffset[1][2] = -60.0*math.pi/180
    thetaoffset[2][0] = +13.0*math.pi/180  #
    thetaoffset[2][1] = +20.*math.pi/180
    thetaoffset[2][2] = -64.0*math.pi/180
    #Leg Set 2
    thetaoffset[3][0] = 9.*math.pi/180   #
    thetaoffset[3][1] = +24*math.pi/180
    thetaoffset[3][2] = -54*math.pi/180
    thetaoffset[4][0] = -3.*math.pi/180  #
    thetaoffset[4][1] = +23*math.pi/180
    thetaoffset[4][2] = -61.2*math.pi/180
    thetaoffset[5][0] = +19.*math.pi/180 #
    thetaoffset[5][1] = 29*math.pi/180
    thetaoffset[5][2] = -50*math.pi/180
    for i in range(0,6):
        for j in range(0,3):
            newtheta[i][j] = oldtheta[i][j] + thetaoffset[i][j]
        reverse =  -newtheta[i][2]
        newtheta[i][2] = reverse
    return newtheta



# This function takes in thetax, thetay, and Zmid values and determines which servo angles are needed, which are returned as theta - a 2d array
# The math behind it was done in mathematica file 'Theta Angles Working.nb'
def plattotheta(thetax, thetay, Zmid,xpostemp, ypostemp):
    
    #Initialise some variables:
    zup = [0. for x in range(6)]
    xpos = [0. for x in range(6)]
    ypos = [0. for x in range(6)]
    thetamodel = [[0. for x in range(3)] for x in range(6)]


    if whichleg ==0:
        thetax = thetax - 0.2*math.pi/180
        thetay = thetay - 0.2*math.pi/180
    else:
        thetax = thetax - 1*math.pi/180
        thetay = thetay + 1*math.pi/180
    #Find the wanted positions for the grounded legs
    for i in range((3*whichleg),(3*whichleg + 3)):
        zup[i] = 0
        xpos[i] = xpostemp - xposinit + xinit[i]
        ypos[i] = ypostemp - yposinit + yinit[i]
 

    #Determine how high the up legs should be
    zupmax = 0.05
    #rinit = math.sqrt(xinit[i]**2. + yinit[i]**2.)
    #rpos = math.sqrt(xpos[i]**2. + ypos[i]**2.)
    reach = math.sqrt((xpostemp-xposinit)**2 + (ypostemp - yposinit)**2)
    
    zuptemp = zupmax/bounds*(0 + bounds - reach)
    #Find the wanted positions for the up legs
    for i in range(3*(1-whichleg),3*(2-whichleg)):
        zup[i] = zuptemp
        xpos[i] = xinit[i]
        ypos[i] = yinit[i]
    #print(xpos)
    #print(ypos)
    
    #Go through each leg and determine the thetas each motor should have
    for i in range(0,6):
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
        shouldbezero = yl*math.cos(theta0)*math.cos(thetay)-xl*math.cos(thetax)*math.sin(theta0)+zl*(-math.sin(theta0)*math.sin(thetax)+math.cos(theta0)*math.sin(thetay))
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
        

    #Add the thetaoffset to the final value
    theta = offsettheta(thetamodel)

    return theta


# Initialise the legs by putting them all up.
def ssc_init():
    #global y
    global zmidps
    inittheta = [[0. for x in range(3)] for x in range(6)]
    for i in range(0,6):
        inittheta[i][1] = -40*math.pi/180
        inittheta[i][2] = +110*math.pi/180
    theta = offsettheta(inittheta)
    tossc(theta)
    
    #Find current r value to determine xinit and yinit. Assuming phi = 0
    for i in range (0,6):
        rth = l1*math.cos(inittheta[i][1]) + l2*math.cos(inittheta[i][1] + inittheta[i][2])
        xinit[i] = math.cos(-legdiff[i])*(r + l0 + rth)
        yinit[i] = math.sin(-legdiff[i])*(r + l0 + rth)
    print(xinit[4], yinit[4])
    #Determine height of the middle of the upper platform so it can be used in psmode() as a midpoint for the height
    zmidps = h + l1*math.sin(inittheta[i][1]) + l2*math.sin(inittheta[i][1] + inittheta[i][2])
    print(zmidps)
    print ("TIPSY INITIALISED")


#Emergency function to put the motors in a default position. 
def NO():
    s.write("#0 P1500 #1 P1500 #2 P1500 #4 P1500 #5 P1500 #6 P1500 #8 P1500 #9 P1500 #10 P1500 #16 P1500 #17 P1500 #18 P1500 #20 P1500 #21 P1500 #22 P1500 #24 P1500 #25 P1500 #26 P1500 T100 \r")
    
def MAX():
    s.write("#0 P2400 #1 P2400 #2 P2400 #4 P2400 #5 P2400 #6 P2400 #8 P2400 #9 P2400 #10 P2400 #16 P2400 #17 P2400 #18 P2400 #20 P2400 #21 P2400 #22 P2400 #24 P2400 #25 P2400 #26 P2400 T100 \r")

def MIN():
    s.write("#0 P600 #1 P600 #2 P600 #4 P600 #5 P600 #6 P600 #8 P600 #9 P600 #10 P600 #16 P600 #17 P600 #18 P600 #20 P600 #21 P600 #22 P600 #24 P600 #25 P600 #26 P600 T100 \r")


#Basic PDI Control system. Put in the error and it returns the thetax and thetay TIPSY needs. Edit the Ki, Kp, and Kd.
def PDIcontrolsystem(errorx,errory, timeint, Ki, Kp, Kd):
    #Kix = 1 #0.7
    #Kpx = 0 #0.1
    #Kdx = 1.4 #1.4
    #Kiy = 1 #0.7
    #Kpy = 0 #0.1
    #Kdy = 1.4 #1.4
    
    Kix = Ki #0.7
    Kpx = Kp #0.1
    Kdx = Kd #1.4
    Kiy = Ki #0.7
    Kpy = Kp #0.1
    Kdy = Kd #1.4
    integralx = 0
    integraly = 0
    derivativepointsx = 0
    derivativepointsy = 0
    
    global ballx, bally
    for i in range(0,span-1):
        ballx[span-1-i] = ballx[span-2-i]
        bally[span-1-i] = bally[span-2-i]
    ballx[0] = errorx
    bally[0] = errory

    for i in range(0,span):
        integralx = integralx + ballx[i]*timeint
        integraly = integraly + bally[i]*timeint
    pointstoderive = 4
    for i in range(0,pointstoderive):
        derivativepointsx = derivativepointsx + (ballx[i] - ballx[i+1])/timeint
        derivativepointsy = derivativepointsy + (bally[i] - bally[i+1])/timeint
        
    derivativex = derivativepointsx/(pointstoderive)
    derivativey = derivativepointsy/(pointstoderive)
    #print("derivativex", derivativex, "integralx", integralx)
    thetax = Kpx*errorx + Kix*integralx + Kdx*derivativex
    thetay = Kpy*errory + Kiy*integraly + Kdy*derivativey
    #print("thetax", thetax)

    return thetax, thetay


#Start playstation controller commands
def psmode():
    while 1:
        #-------------Set up the first variables
        global whichleg, xposinit, yposinit, zoffset    #These variables will be changed globally
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
            Zmid = zmidps*((-joystick.get_axis(12) + joystick.get_axis(13))/3+1)
            #print((-joystick.get_axis(12) + joystick.get_axis(13)+2)/2)

            #----------------Thetax and Thetay input
            thetax = joystick.get_axis(2)*0.3
            thetay = -joystick.get_axis(3)*0.3

            #---------------Walking algorithms
            xdirection = -joystick.get_axis(0)*0.002
            ydirection = joystick.get_axis(1)*0.002
            xpos,ypos = walking(xdirection, xpos, ydirection, ypos,uneven)
            

            
            #----------------Send everything to the servo controller
            theta = plattotheta(thetax,thetay,Zmid,xpos,ypos)
            tossc(theta)
            
            #----------------Uneven Terrain Algorithm. Only turns on when x is toggled
            #If xbutton is pressed, toggle uneven constant
            if xbutton == 1 and uneven == 1 and uneventimer > 50:
                uneven = 0
                print("turn off uneven")
                zoffset = [0. for x in range(6)]
                uneventimer = 0
            else:
                if xbutton == 1 and uneven == 0 and uneventimer > 50:
                    uneven = 1
                    print("turn on uneven")
                    uneventimer = 0
            uneventimer = uneventimer+1
            #If uneven = 1, start uneven terrain  
            if uneven == 1:
                flushbuffer()
                while True:
                        if arduino.inWaiting() > 0:
                            c = arduino.read()
                            if c == '<':
                                break
                value = ''
                while True:
                    if arduino.inWaiting() > 0:
                        c = arduino.read()
                        if c == '>':
                            break
                        value = value + c
                #print(value)

                
                #b = arduino.readline()
                footsensor = [0 for x in range(6)]
                    
                for i in range(6):
                    try:
                        footsensor[i] = int(value[i])
                    except ValueError:
                        footsensor[i] = 0
                        print("ValueError on", i)
                    except IndexError:
                        footsensor[i] = 0
                        print("Indexerror on", i)
                
                for j in range(3*(1-whichleg),3*(2-whichleg)):
                    if footsensor[j] == 1:
                        for i in range(3*(1-whichleg),3*(2-whichleg)):
                            counter = 0
                            while footsensor[i] == 0 and counter < 30:
                                zoffset[i] = zoffset[i] + 0.0001 #Increase the offset until the leg hits the ground
                                theta = plattotheta(thetax,thetay,Zmid,xpos,ypos) #With the new zoffset run the calculations again
                                tossc(theta)#Find a way to just run the calculations for the one leg to reduce calculation times.
                                counter = counter+1
                                flushbuffer()
                                while True:
                                        if arduino.inWaiting() > 0:
                                            c = arduino.read()
                                            if c == '<':
                                                break
                                value = ''
                                while True:
                                    if arduino.inWaiting() > 0:
                                        c = arduino.read()
                                        if c == '>':
                                            break
                                        value = value + c
                                footsensor = [0 for x in range(6)]
                                    
                                for h in range(6):
                                    try:
                                        footsensor[h] = int(value[h])
                                    except ValueError:
                                        footsensor[h] = 0
                                        print("ValueError on", h)
                                    except IndexError:
                                        footsensor[h] = 0
                                        print("Indexerror on", h)
                                
                            #if counter == 50:
                                #print("WE ABORTED BECAUSE IT WAS RUNNING A LOOP")
                            #print("finished moving down", i, zoffset[i], counter)
                            
                        #SWITCH UP AND DOWN LEGS and set new offsets
                        zup = 0.06*(1-((xpos - xposinit)**2 + (ypos - yposinit)**2)/(bounds*bounds)) + Zmid * ((xpos - xposinit)**2 + (ypos - yposinit)**2)/(bounds*bounds)
                        if (whichleg == 0):
                            whichleg = 1
                            for i in range(0,3):
                                zoffset[i] = 0
                            for i in range(3,6):
                                zoffset[i] = - Zmid + (zoffset[i] + zup)
                        else:
                            whichleg = 0
                            for i in range(0,3):
                                zoffset[i] = - Zmid + (zoffset[i] + zup)
                            for i in range(3,6):
                                zoffset[i] = 0
                        #print(zoffset)
                        xposinit = xpos
                        yposinit = ypos
                        theta = plattotheta(thetax,thetay,Zmid,xpos,ypos) #With the new zoffset run the calculations again
                        tossc(theta)#Find a way to just run the calculations for the one leg to reduce calculation times.
                        print("SWITCHING THE LEGS BY UNEVEN")
                        time.sleep(0.1)
                        break
                
                #print("ending uneven terrain")
            
            circle = joystick.get_button(13)
            startbutton = joystick.get_button(3)
            xbutton = joystick.get_button(14)
            square = joystick.get_button(15)
        square = 0

        #Control System movement mode (circle is pressed)
        timeinitial = time.time()
        timearray = []
        xarray = []
        yarray = []
        xposarray = []
        yposarray = []
        timelast = time.time()
        while (square == 0) and (startbutton == 0):
            #print("Start control")
            pygame.event.pump()
            
            #---------------Walking algorithms
            xdirection = -joystick.get_axis(0)*0.002
            ydirection = joystick.get_axis(1)*0.002
            #ydirection = 0
            #xdirection = 0
            xpos,ypos = walking(xdirection, xpos, ydirection, ypos,uneven)
            #Zmid = zmidps*(-joystick.get_axis(12)*0.2+1)
            #----------------Height calculation
            Zmid = zmidps*((-joystick.get_axis(12) + joystick.get_axis(13))/3+1)
            #print((-joystick.get_axis(12) + joystick.get_axis(13)+2)/2)

            #----------------PID Control System for thetax and thetay
            tempposition = []
            flushbuffer()
            while True:
                if arduino.inWaiting() > 0:
                    c = arduino.read()
                    if c == '[':
                        break
            value = ''

            while True:
                if arduino.inWaiting() > 0:
                    c = arduino.read()
                    if c == ']':
                        break
                    value = value + c
            str = value
            tempposition = [float(x) for x in str.split()]
            thetax = -tempposition[0]
            thetay = -tempposition[1]
            #Now position for graphs
            while True:
                if arduino.inWaiting() > 0:
                    c = arduino.read()
                    if c == '{':
                        break
            value = ''

            while True:
                if arduino.inWaiting() > 0:
                    c = arduino.read()
                    if c == '}':
                        break
                    value = value + c
            str = value
            tempposition = [float(x) for x in str.split()]
            position = [0, 0]
            position[0] = tempposition[0]
            position[1] = tempposition[1]
            #print(thetax,thetay)
            #touchplate is on backwards, change them to negative:
            
            #position[0] = -tempposition[0]
            #position[1] = -tempposition[1]
            #print(position)
            """
            xposgoal = 0
            yposgoal = 0
            errorx = position[0] - xposgoal
            errory = position[1] - yposgoal
            timeint = 0.12
            
            timearray.append(time.time() - timeinitial)
            xarray.append(errorx)
            yarray.append(errory)
            xposarray.append(xpos)
            yposarray.append(ypos)
            """
            Kitemp = joystick.get_axis(3)*0.0001
            Kptemp = joystick.get_button(4)*0.0005 - joystick.get_button(6)*0.0005
            Kdtemp = joystick.get_axis(2)*0.0001
            Ki = 0 #Ki + Kitemp
            Kp = Kp + Kptemp
            Kd = Kd + Kdtemp
            #print(Kp, Ki, Kd)
            #print(thetax,thetay)
            Kforarduino = "P%s,I%s,D%s," % (Kp, Ki, Kd)
            triangle = joystick.get_button(12)
            if triangle == 1:
                #Kforarduino = "Q"
                Kp = 0.019500
                Ki = 0
                Kd = 0.02104781
                Kforarduino = "P%s,I%s,D%s," % (Kp, Ki, Kd)
                
            arduino.flushOutput()
            arduino.flushInput()
            arduino.write(Kforarduino) #Send the PID gain constants to the arduino to use
            #thetax,thetay = PDIcontrolsystem(errorx,errory, timeint, Ki, Kp, Kd)

            #print(thetax, thetay)
            #print("positionx:", position[0], "thetax:", thetax*180/math.pi)
            #print(thetax, thetay)
            #timenow = time.time() - timelast
            #print(timenow)
            #timelast = time.time()
            

            #----------------Send everything to the servo controller
            theta = plattotheta(thetax,thetay,Zmid,xpos,ypos)
            tossc(theta)

            #----------------Uneven Terrain Algorithm. Only turns on when x is toggled
            #If xbutton is pressed, toggle uneven constant
            if xbutton == 1 and uneven == 1 and uneventimer > 50:
                uneven = 0
                print("turn off uneven")
                zoffset = [0. for x in range(6)]
                uneventimer = 0
            else:
                if xbutton == 1 and uneven == 0 and uneventimer > 50:
                    uneven = 1
                    print("turn on uneven")
                    uneventimer = 0
            uneventimer = uneventimer+1
            #If uneven = 1, start uneven terrain  
            if uneven == 1:
                flushbuffer()
                while True:
                        if arduino.inWaiting() > 0:
                            c = arduino.read()
                            if c == '<':
                                break
                value = ''
                while True:
                    if arduino.inWaiting() > 0:
                        c = arduino.read()
                        if c == '>':
                            break
                        value = value + c
                #print(value)

                
                #b = arduino.readline()
                footsensor = [0 for x in range(6)]
                    
                for i in range(6):
                    try:
                        footsensor[i] = int(value[i])
                    except ValueError:
                        footsensor[i] = 0
                        print("ValueError on", i)
                    except IndexError:
                        footsensor[i] = 0
                        print("Indexerror on", i)
                
                for j in range(3*(1-whichleg),3*(2-whichleg)):
                    if footsensor[j] == 1:
                        for i in range(3*(1-whichleg),3*(2-whichleg)):
                            counter = 0
                            while footsensor[i] == 0 and counter < 30:
                                zoffset[i] = zoffset[i] + 0.001 #Increase the offset until the leg hits the ground
                                theta = plattotheta(thetax,thetay,Zmid,xpos,ypos) #With the new zoffset run the calculations again
                                tossc(theta)#Find a way to just run the calculations for the one leg to reduce calculation times.
                                counter = counter+1
                                flushbuffer()
                                while True:
                                        if arduino.inWaiting() > 0:
                                            c = arduino.read()
                                            if c == '<':
                                                break
                                value = ''
                                while True:
                                    if arduino.inWaiting() > 0:
                                        c = arduino.read()
                                        if c == '>':
                                            break
                                        value = value + c
                                footsensor = [0 for x in range(6)]
                                    
                                for h in range(6):
                                    try:
                                        footsensor[h] = int(value[h])
                                    except ValueError:
                                        footsensor[h] = 0
                                        print("ValueError on", h)
                                    except IndexError:
                                        footsensor[h] = 0
                                        print("Indexerror on", h)
                                
                            #if counter == 50:
                                #print("WE ABORTED BECAUSE IT WAS RUNNING A LOOP")
                            #print("finished moving down", i, zoffset[i], counter)
                            
                        #SWITCH UP AND DOWN LEGS and set new offsets
                        zup = 0.06*(1-((xpos - xposinit)**2 + (ypos - yposinit)**2)/(bounds*bounds)) + Zmid * ((xpos - xposinit)**2 + (ypos - yposinit)**2)/(bounds*bounds)
                        if (whichleg == 0):
                            whichleg = 1
                            for i in range(0,3):
                                zoffset[i] = 0
                            for i in range(3,6):
                                zoffset[i] = - Zmid + (zoffset[i] + zup)
                        else:
                            whichleg = 0
                            for i in range(0,3):
                                zoffset[i] = - Zmid + (zoffset[i] + zup)
                            for i in range(3,6):
                                zoffset[i] = 0
                        #print(zoffset)
                        xposinit = xpos
                        yposinit = ypos
                        theta = plattotheta(thetax,thetay,Zmid,xpos,ypos) #With the new zoffset run the calculations again
                        tossc(theta)#Find a way to just run the calculations for the one leg to reduce calculation times.
                        print("SWITCHING THE LEGS BY UNEVEN")
                        time.sleep(0.1)
                        break
                
                #print("ending uneven terrain")
            
            

            square = joystick.get_button(15)
            startbutton = joystick.get_button(3)
            xbutton = joystick.get_button(14)
        circle = 0
        xbutton = 0
        """
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
        ax1.plot(timearray,xarray, 'r', timearray, yarray, 'b')
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('x position (m) (red), y position (m) (blue)')
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111)
        ax2.plot(timearray,xposarray, 'r', timearray, yposarray, 'b')
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('x position (red), y position (blue)')
        """
        print("PS mode ending")
        print(Kp, Ki, Kd)
        #plt.show()

    


#This function just switches the legs and provides the new positions for the robot as it moves
def walking(xdirection, xpos, ydirection, ypos, uneven):
    global xposinit, yposinit, whichleg
    xpos = xpos + xdirection
    ypos = ypos + ydirection
    if uneven == 0:
        reach = (xpos-xposinit)**2 + (ypos - yposinit)**2
        #Determine if the bot is moving too far, and if so, change legs to continue walking
        if (reach > bounds*bounds):
            xposinit = xpos
            yposinit = ypos
            #SWITCH UP AND DOWN LEGS
            if (whichleg == 0):
                whichleg = 1
                zoffset[0] = 0
                zoffset[1] = 0
                zoffset[2] = 0
            else:
                whichleg = 0
                zoffset[3] = 0
                zoffset[4] = 0
                zoffset[5] = 0
            print("SWITCHING THE LEGS BY WALKING")
            time.sleep(0.1)
    

    return xpos, ypos


#Algorithm that allows the spider to walk on uneven terrain. It does this by triggering when an up foot sensor
#has a value of 1 and lowering the other feet until they also trigger. It then takes in the values of the z's
#and applies them on top of the needed z in the next step when those feet are on the ground.

        
            
ssc_init()
psmode()
