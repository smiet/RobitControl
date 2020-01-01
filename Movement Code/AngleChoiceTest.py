import matplotlib.pyplot as plt
import math
import numpy as np


pi = math.pi
arcangle = 3*pi/4
legminbound = 0.05
legmaxbound = 0.18
upleg = 0

plottingarrayx = []
plottingarrayy = []
anglearray = []
#Simple function that takes in any two angles and returns the actual difference, taking into account wrapping around 2pi
def anglediff(angle1,angle2):
    newangle = angle1 - angle2
    while newangle > pi:
        newangle = newangle - 2*pi
    while newangle < -pi:
        newangle = newangle + 2*pi
    return newangle    
    #return (angle1 - angle2 + pi/2 + pi) % pi - pi/2



#Test that the angles and positions are correct.
#for i in range(1:5:360):
for i in range(1,360,1):
    velocangle = i*pi/180
    #print(velocangle)

    legangleoffset = [pi/2 - arcangle/2, pi - arcangle/2, -arcangle/2,-pi/2 - arcangle/2 ]#put outside of function.
    #legangleoffset = [0,0,0,0 ]#put outside of function. This is to make sure the local y axis is in the right direction
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
    plottingarrayx.append(uplegx)
    plottingarrayy.append(uplegy)
    anglearray.append(legangle)
                 
t = range(0,len(plottingarrayx))

plt.scatter(plottingarrayx,plottingarrayy,c = t )
plt.ylim((-legmaxbound, legmaxbound))
plt.xlim((-legmaxbound,legmaxbound))
plt.show()
#print(np.asarray(anglearray))
#print(np.asarray(plottingarrayx))
print(minangle7)
print(maxangle7)

