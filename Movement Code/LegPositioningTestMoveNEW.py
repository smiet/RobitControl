import serial
import math
import pygame
import time
import numpy as np

pi = math.pi
arcangle = 3*pi/4 #Should be the angle limits of the leg for the robot. Change this.
legmaxbound = 0.18 #How far a leg can extend. Change this
legminbound = 0.05 #How close a leg can contract. Change this
#PROBLEMS RIGHT NOW

#Something is wrong with the angle bound calculation. When the leg moves into a new position it should be far from abound but the next loop its there again??
#Motors arent responding. Swap out a different arbotix to check.
#Math domain is somehow appearing at certain velocity angles...probably related to the above angle bound bug. Check when motors work.

#--------------------------------------------------------------------------------------------
#--------------------------Functions to be used
#Function takes in the leg position vector (in the legframe: legposLF) and converts the desired position into angles for the motors
def legpostotheta(legpos):
    L1 = math.sqrt(legpos[0]*legpos[0]+legpos[1]*legpos[1])
    L = math.sqrt(legpos[2]*legpos[2]+(L1-l0)**2)
    gamma = math.atan2(legpos[1],legpos[0])
    alpha = math.acos(legpos[2]/L) + math.acos((l2*l2-l1*l1-L*L)/(-2*l1*L))
    beta = math.acos((L*L-l2*l2-l1*l1)/(-2*l2*l1))
    return (gamma,alpha,beta)

#Function takes in the desired motor angles and converts them into the format needed for the motor to interpret
def thetatoservo(angle):
    #sscnumber = angle*1024/300*180/math.pi* + 512. #Assuming in radians
    sscnumber = angle*195.56959 + 512. #Assuming in radians    
    if sscnumber > 750:
        sscnumber = 800
        print("YOU'RE TOO HIGH")
    if sscnumber < 250:
        sscnumber = 200
        print("YOU'RE TOO LOW")
    sscnumberout = int(round(sscnumber))
    return sscnumberout

#Function applies the offset to the motor angles from the ideal frame assumed in the math, to that which the motors are preset to have. i.e if the motor
#has a zero position that is off the model, this will fix apply an offset to fix that.
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


#Unsure if this function is still being used.
def bodypostolegpos(bodypos):
    legpos[0] = [initpos + bodypos[0], bodypos[1], bodypos[2]]
    legpos[1] = [initpos + bodypos[1], -bodypos[0], bodypos[2]]
    legpos[2] = [initpos - bodypos[1], bodypos[0], bodypos[2]]
    legpos[3] = [initpos - bodypos[0], -bodypos[1], bodypos[2]]
    return legpos

#Function calculates the area of the triangle drawn by 3 points. x1 and y1 refer to the 2d coordinates of the first point etc.
def area(x1,x2,x3,y1,y2,y3):
    return abs((x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2.0)

#Function takes in the absolute position of the legs and body to check if the body is within the triangle drawn by the leg positions.
#The function determines which leg to not include (usually the upleg) by the variable 'i'.
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

#Function checks if one point is in a triangle drawn by three other points. p_test is the point being checked and p0, p1, and p2 draw the triangle
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


#Explanation of the frames of reference:
    #legposLF: leg position vector where the origin is the first motor of the leg (leg position in the leg frame LF)
    #legposstep: leg position vector where the origin is where the robot starts.
    #bodypos: robot position vector where the origin is where the robot starts
    #delbody: derivative of the bodypos frame
        

#Function transforms the legposstep frame to the legposLF frame. Essentially this takes an absolute position and puts it into the legs frame of reference.
def legposstepTOlegposLF(legposstep,bodypos):
    legpos = [[0. for x in range(3)] for x in range(4)]
    legpos[0] = [legposstep[0][0]-bodypos[0], legposstep[0][1]-bodypos[1]-r, legposstep[0][2]]
    legpos[1] = [legposstep[1][1]-bodypos[1], -(legposstep[1][0]-bodypos[0])-r, legposstep[1][2]]
    legpos[2] = [-(legposstep[2][1]-bodypos[1]), legposstep[2][0]-bodypos[0]-r, legposstep[2][2]]
    legpos[3] = [-(legposstep[3][0]-bodypos[0]), -(legposstep[3][1]-bodypos[1])-r, legposstep[3][2]]
    return legpos


#Function that checks how far away a point is from the line created by two other points and a velocity vector
#dist is the distance along the velocity vector that the bodypos point will intersect the line between the two legpos points.
def distancecheck(legpos1,legpos2,bodypos,veloc):
    #velocnorm = veloc/math.sqrt(veloc[0]*veloc[0]+veloc[1]*veloc[1])
    velocmag = math.sqrt(veloc[0]*veloc[0]+veloc[1]*veloc[1])
    velocnorm = [x / velocmag for x in veloc]
    leglineslopedenom = (legpos2[1]-legpos1[1])
    if leglineslopedenom == 0: #If the line is vertical, do not divide by zero but use a simpler form for the distance
        dist = legpos2[1] - legpos1[1]/velocnorm[1]
    else:
        leglineslope = (legpos2[0]-legpos1[0])/leglineslopedenom #Not really the slope but the inverse
        denominator = leglineslope*velocnorm[1] - velocnorm[0]
        if denominator == 0: #Make sure its not parallel otherwise it will divide by zero. Instead set to a high number
            dist = 100
        else:
            dist = (bodypos[0]-legpos1[0]-leglineslope*(bodypos[1]-legpos1[1]))/denominator
    return dist

#Simple function that takes in any two angles and returns the actual difference, taking into account wrapping around 2pi
def anglediff(angle1,angle2):
    newangle = angle1 - angle2
    while newangle > pi:
        newangle = newangle - 2*pi
    while newangle < -pi:
        newangle = newangle + 2*pi
    return newangle    
    #return (angle1 - angle2 + pi/2 + pi) % pi - pi/2

#Function that finds the optimal position of the upleg based on the velocity vector and the other leg positions
def optimaluplegposition(velocangle,Zup,bodypos,bodyframeadjustment,upleg,legposstep):
    legangleoffset = [pi/2 - arcangle/2, pi - arcangle/2, -arcangle/2,-pi/2 - arcangle/2 ]#put outside of function. This is to make sure the local y axis is in the right direction
    angle = velocangle - legangleoffset[upleg] #Take the arc bound as the local x coordinate in this function. Offset is reapplied at the end

    #Define where the different cases exist for each angle
    maxangle1 = (arcangle/2)
    maxangle2 = pi/2
    maxangle3 = pi/2 + math.acos(legminbound/legmaxbound)/2
    maxangle4 = pi/2 + arcangle/2
    maxangle5 = arcangle + pi/2 - math.acos(legminbound/legmaxbound)/2
    maxangle6 = arcangle + pi/2
    maxangle7 = maxangle1 + pi
    maxangle8 = maxangle2 + pi
    maxangle9 = maxangle3 + pi
    maxangle10 = maxangle4 + pi
    maxangle11 = maxangle5 + pi
    maxangle12 = maxangle6 + pi
    
    minangle2 = maxangle1
    minangle3 = maxangle2
    minangle4 = maxangle3
    minangle5 = maxangle4
    minangle6 = maxangle5
    minangle7 = maxangle6
    minangle8 = maxangle7
    minangle9 = maxangle8
    minangle10 = maxangle9
    minangle11 = maxangle10
    minangle12 = maxangle11
    minangle1 = maxangle12
    legminboundscale = 1.05
    legmaxboundscale = 0.95

    
    #Now begin the decision tree to decide how to place the leg.
    #anglediff's are everywhere to ensure we are always comparing the right angles without pi wrapping being a problem
    if anglediff(angle,minangle1) >= 0 and anglediff(angle,maxangle1) < 0:
        #Interpolate between the angle to determine what angle it should be
        anglemin = arcangle - math.acos(legminbound/legmaxbound)
        anglemax = arcangle/2 + math.asin(legminbound/legmaxbound*math.sin(pi-arcangle/2))
        lowerangle = minangle1
        higherangle = maxangle1
        legangle = anglemin + anglediff(angle,lowerangle)*anglediff(anglemax,anglemin)/anglediff(higherangle,lowerangle) + legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case1')
        
    elif anglediff(angle,minangle2) >= 0 and anglediff(angle,maxangle2) < 0:
        #Interpolate between the angle to determine what angle it should be
        anglemin = arcangle/2 - math.asin(legminbound/legmaxbound*math.sin(pi-arcangle/2))
        anglemax = math.acos(legminbound/legmaxbound)
        lowerangle = minangle2
        higherangle = maxangle2
        legangle = anglemin + anglediff(angle,lowerangle)*anglediff(anglemax,anglemin)/anglediff(higherangle,lowerangle) + legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case2')
        
    elif anglediff(angle,minangle3) >= 0 and anglediff(angle,maxangle3) < 0:
        #Fixed point
        legangle = math.acos(legminbound/legmaxbound)  + legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case3')

    elif anglediff(angle,minangle4) >= 0 and anglediff(angle,maxangle4) < 0:
        #Interpolate between the angle to determine what angle it should be
        anglemin = math.acos(legminbound/legmaxbound)
        anglemax = arcangle
        lowerangle = minangle4
        higherangle = maxangle4
        legangle = anglemin + anglediff(angle,lowerangle)*anglediff(anglemax,anglemin)/anglediff(higherangle,lowerangle) + legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case4')

    elif anglediff(angle,minangle5) >= 0 and anglediff(angle,maxangle5) < 0:
        #Fixed point
        legangle =  arcangle + legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case5')

    elif anglediff(angle,minangle6) >= 0 and anglediff(angle,maxangle6) < 0:
        #Interpolate between the angle to determine what radius it should be
        legangle = legangleoffset[upleg] + arcangle
        radiusmin = legmaxboundscale*legmaxbound
        radiusmax = legminboundscale*legminbound
        lowerangle = minangle6
        higherangle = maxangle6
        legradius = radiusmin + anglediff(angle,lowerangle)*(radiusmax - radiusmin)/anglediff(higherangle,lowerangle)
        #print('case6')
        
    elif anglediff(angle,minangle7) >= 0 and anglediff(angle,maxangle7) < 0:
        #Fixed point
        legangle = legangleoffset[upleg] + arcangle
        legradius = legminboundscale*legminbound
        #print('case7')
        
    elif anglediff(angle,minangle8) >= 0 and anglediff(angle,maxangle8) < 0:
        #Fixed point
        legangle = legangleoffset[upleg]
        legradius = legminboundscale*legminbound
        #print('case8')
        
    elif anglediff(angle,minangle9) >= 0 and anglediff(angle,maxangle9) < 0:
        #Interpolate between the angle to determine what radius it should be
        legangle = legangleoffset[upleg]
        radiusmin = legminboundscale*legminbound
        radiusmax = legmaxboundscale*legmaxbound
        lowerangle = minangle9
        higherangle = maxangle9
        legradius = radiusmin + anglediff(angle,lowerangle)*(radiusmax - radiusmin)/anglediff(higherangle,lowerangle)
        #print('case9')
        
    elif anglediff(angle,minangle10) >= 0 and anglediff(angle,maxangle10) < 0:
        #Fixed point
        legangle = legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case10')
        
    elif anglediff(angle,minangle11) >= 0 and anglediff(angle,maxangle11) < 0:
        #Interpolate between the angle to determine what angle it should be
        anglemin = 0
        anglemax = arcangle - math.acos(legminbound/legmaxbound)
        lowerangle = minangle11
        higherangle = maxangle11
        legangle = anglemin + anglediff(angle,lowerangle)*anglediff(anglemax,anglemin)/anglediff(higherangle,lowerangle) + legangleoffset[upleg]
        legradius = legmaxboundscale*legmaxbound
        #print('case11')
        
    elif anglediff(angle,minangle12) >= 0 and anglediff(angle,maxangle12) < 0:
        #Fixed point
        legangle = legangleoffset[upleg] + arcangle - math.acos(legminbound/legmaxbound)
        legradius = legmaxboundscale*legmaxbound
        #print('case12')

    #Calculate the position of the new upleg
    uplegx = legradius*math.cos(legangle)    
    uplegy = legradius*math.sin(legangle)
    optimalupleg = [uplegx, uplegy, -Zup]
    optimalupleg = np.asarray(optimalupleg) + bodypos + bodyframeadjustment[upleg] #Add offset for leg hip position and robot position
               
    return optimalupleg

        
#--------------------------------------------------------------------------------------------
#--------------------------Main loop
#Basic walking function which moves the robot by changing the legs and giving new motor commands
def walking(boop):
    legpos = [[0. for x in range(3)] for x in range(4)]
    legposLF = [[0. for x in range(3)] for x in range(4)]
    legposstep = [[0. for x in range(3)] for x in range(4)]
    bodyframeadjustment = [[0. for x in range(3)] for x in range(4)]
    Zmid = 0.12
    
    bodyposinit = [0.0, 0.0, Zmid]
    bodypos = bodyposinit
    legposstep[0] = [0.0, initpos, Zmid]
    legposstep[1] = [-initpos, 0.0, Zmid]
    legposstep[2] = [initpos, 0, Zmid]
    legposstep[3] = [0, -initpos, Zmid]
    bodyframeadjustment[0] = [0.0, r, 0.0]
    bodyframeadjustment[1] = [-r, 0.0, 0.0]
    bodyframeadjustment[2] = [r, 0.0, 0.0]
    bodyframeadjustment[3] = [0.0, -r, 0.0]
    legposstep = np.asarray(legposstep) + bodyframeadjustment #Adds the distance between leg motor and body center
    legposstepINIT = legposstep #Save this variable for now....temporary    
    #Adjust the initial leg position so it isnt in a singularity
    legposstep[1] = np.asarray(legposstep[1]) + [-0.01, 0.0, 0.0]
    legposstep[3] = np.asarray(legposstep[3]) + [0.04, 0.0, 0.0]
    
    j = 1
    rev = 0
    upleg = 0 #This is the leg that is initially up
    Zup = 0.05 #The amount the upleg is raised from the floor
    tempmove = 1
    from operator import add

    time.sleep(1)
    print("waiting")

    
    while 1:
        #time.sleep(.2)
        print('NEW STEP', bodypos)
        delbody = [float(j*tempmove)/300,-float(j*tempmove)/300,0.0] #Test code for the direction the robot is given for movement
        bodypos = np.asarray(bodypos) + np.asarray(delbody) #Move the robot body by the directed amount
        tempmove = 1
        velocitythreshold = 0.00001 #If velocity is above this value then it will move. Otherwise hold still and be stable.
        
        #If there is no velocity vector given, it should remain still and put all legs on the ground for stability.
        if (delbody[0]*delbody[0] + delbody[1]*delbody[1] > velocitythreshold):
            #WALK
            # -------------------- Give the target position to the upleg
            #The upleg should ideally move to where it will be most useful - that is where it will be on the ground the longest before switching
            #and well within a future stable zone. To do this a tangent is drawn on the minimum bound circle in the direction of the
            #velocity vector and the leg is placed by the outer bound circle. There are 4 locations where this can be. Two of them are in the
            #wrong direction, and of the other two one will have a location within stability for longer. This one is chosen.

            #Find the locations where the leg should be
            #Find the optimal locations of the upleg and pick the best one.
            velocangle = math.atan2(delbody[1],delbody[0])
            legposstep[upleg] = optimaluplegposition(velocangle,Zup,bodypos,bodyframeadjustment,upleg,legposstep)
            

            #Find the leg frame positions of all legs       
            legposLF = legposstepTOlegposLF(legposstep,bodypos)
            print('upleg is', upleg)
            


            # -------------------- Check if leg needs to be changed
            #Leg choice algorithm is initiated by a leg-change event
            #Leg change is prompted by one of five events happening:
            #               A leg reaches its maximum bound
            #               A leg reaches its minimum bound
            #               A leg reaches an angle bound
            #               The center of mass is about to cross a triangle of stability to another triangle of stability
            #               The center of mass is about to cross a triangle of stability to no stability
            #If multiple of these are about to occur, the instability event is given the highest priority due to the buffer on the bounds
            #Once the new upleg is chosen, on the next loop the upleg will move to its optimal leg placement
            #The new upleg is then given the command to move to the optimal leg placement for the next step.
            #Check if leg needs to change. If so set legchange to 1
            legchange = 0
            
            #Check if legs are about to be unstable
            legcheck = masscentercheck(legposstep,bodypos,upleg)
            if legcheck == False:
                legchange = 2
                print('leg change due to unstable check')
                       
            #Check bound limits of the legs if the legs are about to be where they shouldn't
            for i in range(4):
                legreachsq = (legposLF[i][0]*legposLF[i][0] + legposLF[i][1]*legposLF[i][1]) #Square of leg reach
                #Check if legs are outside of maximum reach
                if (legmaxbound*legmaxbound) < legreachsq:
                    if legchange == 0: #Priority set for unstable to be more important to change
                        legchange = 1
                        legtochange = i
                    print('leg change due to max bound reached on leg', i)
                #Check if legs are inside of minimum reach
                if (legminbound*legminbound) > legreachsq:
                    if legchange == 0: #Priority set for unstable to be more important to change
                        legchange = 1
                        legtochange = i
                    print('leg change due to min bound reached on leg', i)
                #Check if legs are within the angle bounds
                if math.atan2(legposLF[i][1],legposLF[i][0]) < pi/2 - arcangle/2 or math.atan2(legposLF[i][1],legposLF[i][0]) > pi/2 + arcangle/2:
                    if legchange == 0: #Priority set for unstable to be more important to change
                        legchange = 1
                        legtochange = i
                    print('leg change due to angle bound reached on leg', i)  


            #If leg change is due to a bound being reached, change that one and continue
            if legchange == 1:
                upleg = legtochange
                #print('new upleg is', upleg)
                downlegs = [0,1,2,3]
                downlegs.remove(upleg) #This creates the indices for the downleg options
                for i in downlegs:
                    #print(downlegs)
                    legposstep[i][2] = Zmid
                    
            #If leg change is due and because of stability, decide best up leg. Is this due to body moving towards or away intersection cross?
            #If this is due to the leg moving towards, find the best available leg to move up to maintain stability.
            #If this is due to the leg moving away, find the leg which will provide the best stability when it has moved to the best position it can have
            if legchange == 2:
                uplegoptionarray = []
                corrdistarray = []
                #First determine which two legs are options for stable positions. There should be only two.
                for i in range(4):
                    legcheckscan = masscentercheck(legposstep,bodypos,i)
                    if legcheckscan == True:
                        uplegoptionarray.append(i)
                        #print(i,' passed the uplegcheck')
                #Once leg options are found, determine which will be stable for longer by checking the distance between the body position point and the
                #edge of the triangle of stability option along the velocity vector.

                #Check if moving towards or away from regions of stability. If no upleg options are found, it is moving away:
                if not uplegoptionarray:
                    #Robot is about to fall over and have no other leg to switch to. Find which leg would be optimal
                    #For now, pick the leg closest to the vector of direction
                    print("robot is moving from regions of stability. Picking closest leg")
                    upleg = 1
                    print(velocangle)
                    if velocangle > -2.35619:
                        upleg = 3
                        if velocangle > -0.7853981:
                            upleg = 2
                            if velocangle > 0.7853981:
                                upleg = 0
                #If uplegoptionarray contains something, it is moving towards regions of stability.
                else:
                    print("robot is moving into regions of stability. Picking best leg")
                    for i in uplegoptionarray:
                        #There are three lines formed by the leg combination, but only one that would be intersected along the velocity vector (positive direction). Find this line.
                        #First find which legs are considered down
                        uplegoption = i
                        downlegoptions = [0,1,2,3]
                        downlegoptions.remove(uplegoption) #This creates the indices for the downleg options
                        #Determine which two legs create the line that will be intersected
                        #print(delbody)
                        distchecktemp1 = distancecheck(legposstep[downlegoptions[0]],legposstep[downlegoptions[1]],bodypos,delbody)
                        distchecktemp2 = distancecheck(legposstep[downlegoptions[1]],legposstep[downlegoptions[2]],bodypos,delbody)
                        distchecktemp3 = distancecheck(legposstep[downlegoptions[2]],legposstep[downlegoptions[0]],bodypos,delbody)
                        correctdist = max([distchecktemp1, distchecktemp2, distchecktemp3])
                        #print(correctdist)
                        corrdistarray.append(correctdist)
                    #Choose the upleg that is the furthest from the intersecting line along the velocity vector
                    if corrdistarray[0] > corrdistarray[1]:
                        upleg = uplegoptionarray[0]
                    else:
                        upleg = uplegoptionarray[1]
                #print('new upleg is', upleg)
                #Set the old upleg to the correct z position by changing all. All legs will be down before the new one lifts up next loop.
                downlegs = [0,1,2,3]
                #downlegs.remove(upleg) #This creates the indices for the downleg options
                for i in downlegs:
                    #print(uplegoptionarray)
                    #print(downlegs)
                    legposstep[i][2] = Zmid
                    #Prompt all legs to move down first before lifting the other one up....hackey consider reomving
                
        else:
            #Stay still all legs on ground.
            downlegs = [0,1,2,3]
            for i in downlegs:
                #print(uplegoptionarray)
                #print(downlegs)
                legposstep[i][2] = Zmid

                
        legposLF = legposstepTOlegposLF(legposstep,bodypos) #Refresh the value of legposLF
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
        #print(botmovement)
        """
        if botmovement[1] > 0.0533:
            rev = 1
            tempmove = 3
            print("REVERSING NEXT")
            print("Bot position", botmovement)
            #upleg = 2
        if botmovement[1] < -0.0533:
            rev = 0
            tempmove = 3
            print("REVERSING NEXT")
            print("Bot position", botmovement)
            #upleg = 1
        if rev == 0:
            j = 1
        else:
            j = -1
        """
        time.sleep(0.04)


#--------------------------------------------------------------------------------------------
#--------------------------Initialize the robot code
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
initpos=0.13
#0.27166 is the limit of the leg in the x and y with 5 cm z position. 19.2cm for 45 deg both legs


Zmid = 0.12
walking(1)
    
