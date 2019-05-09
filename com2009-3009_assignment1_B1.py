#!/usr/bin/env python3

'''COM2009 ev3dev Assignment2'''



import os

import sys

import time

import ev3dev.ev3 as ev3

from random import *



'''Provided Functions'''



def debug_print(*args, **kwargs):

    '''Print debug messages to stderr.



    This shows up in the output panel in VS Code.

    '''

    print(*args, **kwargs, file=sys.stderr)





def reset_console():

    '''Resets the console to the default state'''

    print('\x1Bc', end='')





def set_cursor(state):

    '''Turn the cursor on or off'''

    if state:

        print('\x1B[?25h', end='')

    else:

        print('\x1B[?25l', end='')





def set_font(name):

    '''Sets the console font



    A full list of fonts can be found with `ls /usr/share/consolefonts`

    '''

    os.system('setfont ' + name)



''' End provided functions '''





def debug_printer(var_name, var_val):

    debug_print(var_name)

    debug_print(var_val)







def actuate(m_left, m_right, v, dv, leftFlip, rightFlip):

    ''' Function to actuate motors 



    Takes 2 motors as inputs, v for base speed, dv for diff 

    returns nil

    '''



    left = (v+dv)*leftFlip

    right = (v-dv)*rightFlip



    '''debug_printer("Fed into motor left:", (left))

    debug_printer("Fed into right motor:", (right))'''



    m_left.run_direct(duty_cycle_sp=left) 

    m_right.run_direct(duty_cycle_sp=right)



def get_response(kp, ki, kd, dt, cur_error, prev_error, integral):

    '''Get PID response'''

    integral = 0

    integral+= cur_error * dt

    derivative = (cur_error - prev_error) / dt



    return (kp * cur_error) + (ki * integral) + (kd * derivative)





''' hk error metric - difference of sensors '''

def hk_metric(l_dist, r_dist):

    return (l_dist-r_dist)





def theo_metric(l_dist, r_dist):

    return ((l_dist+r_dist)/2)



def theo_jose_hybrid_metric(l_dist, r_dist):

    if l_dist > 2500:

        return (r_dist / 2)

    elif r_dist > 2500:

        return (l_dist / 2)

    else:

        return theo_metric(l_dist, r_dist)





# Motor vars

m_left = ev3.LargeMotor('outB')

m_right = ev3.LargeMotor('outC')



# Sensors

us_left = ev3.UltrasonicSensor('in3')

us_right = ev3.UltrasonicSensor('in2')



cs = ev3.ColorSensor('in1')

cs.mode = 'COL-AMBIENT'

gy = ev3.GyroSensor('in4')

gy.mode = 'GYRO-ANG'





global runWalk

global runSearch







def walk(walkTime, lightFarValue, lightInfront):

    sp = -10

    r = 0

    b = 0

    e = r - b

    u = 0

    ic = time.clock() #initial clock time

    dt = 0

    de = 0

    t = 0

    chillMode = 0.08

    TU = 0.45

    KU = 0.17

    KP = 0.6 * KU

    KI = (1.2 * KU) / TU

    KI = KI * dt

    KD = (3 * KU * TU) / 40

    dt = time.clock() - t

    KD = KD/dt

    maxMotorValue = 80

    t_interval = ic

    integral = 0

    m = (r*2)/maxMotorValue

    bMotorValue = 0

    cMotorValue = 0

    turn = 0

    speed = 60

    startTime = time.time()

    currentTime = time.time()

    while (currentTime < (startTime + walkTime) and cs.value() < lightFarValue or lightInfront == True):

        timePassed = currentTime - startTime

        currentTime = time.time()

        t = time.clock() - ic

        e1 = r - b

        integral = integral + (e1 * dt)

        dt = time.clock() - t

        de = e1 - e

        u = (KP*e1) + (KI*integral) + (KD*de)

        turn = m * e1

        if (u < -maxMotorValue):

            u = -maxMotorValue

        elif (u > maxMotorValue):

            u = maxMotorValue

        bMotorValue = (u * -1) - speed

        cMotorValue = u - speed

        if (bMotorValue < -maxMotorValue):

            bMotorValue = -maxMotorValue

        elif (bMotorValue > maxMotorValue):

            bMotorValue = maxMotorValue

        if (cMotorValue < -maxMotorValue):

            cMotorValue = -maxMotorValue

        elif (cMotorValue > maxMotorValue):

            cMotorValue = maxMotorValue

        if cs.value() > 12:
            m_right.run_direct(duty_cycle_sp=int(bMotorValue))
            m_left.run_direct(duty_cycle_sp=int(cMotorValue))
        else:
            m_left.run_direct(duty_cycle_sp=int(bMotorValue))
            m_right.run_direct(duty_cycle_sp=int(cMotorValue))



        e = e1

        b = (us_left.value()*-1) + us_right.value()



def setNewDirection(input):

    if input == 0:
        randomWalkAngle = randint(0, 270)
    else:
        randomWalkAngle = input

    startingAngle = gy.value()
    currentAngle = gy.value()

    while currentAngle < (startingAngle + randomWalkAngle):
        actuate(m_left, m_right, 20, 20, -1, 1)
        currentAngle = gy.value()





def search(lightFarValue):

    global runWalk

    global runSearch

    rotateSpeed = 40

    units = gy.units

    angle = gy.value()

    startingAngle = angle

    currentAngle = angle

    currentCSValue = cs.value()

    startCSValue = cs.value()

    #debug_print(str(angle) + " " + units)

    #debug_print("CS VALUE:" + str(cs.value()))

    
    DL = [1, -1]
    direction = sample(DL, 1)

    while runSearch == True:
        currentAngle = gy.value()
        if gy.value() > currentAngle:

            #if the colour sensor value is triggered then stop searching and start walking
            if cs.value() > lightFarValue:
                debug_print("Sensor stopped search")
                runSearch = False
                runWalk = True

        else:

            #update CS value correctly, fix the robot, find the light, win the competition

            actuate(m_left, m_right, rotateSpeed, rotateSpeed, direction[0], direction[0])

        if currentAngle > (startingAngle + (270 * direction[0] * -1)):
            debug_print("Rotation stopped search")
            setNewDirection(0)
            runSearch = False
            runWalk = True

    if cs.value() > lightFarValue:
        setNewDirection(15)








def main():

    '''Main calling Function'''

    debug_print("Running")

    global runWalk

    global runSearch

    runWalk = True

    runSearch = False

    lightFound = False

    lightCloseValue = 15

    lightFarValue = 10

    while lightFound == False:
        if cs.value() >= lightCloseValue:

            runWalk = False

            runSearch = False

            lightFound = True

            debug_print("LIGHT FOUND!!!")

        elif runWalk == True:

            debug_print("Walk")
            
            startTime = time.time()

            # Random walk time
            randomWalkTime = randint(1, 7)
            debug_print(randomWalkTime)

            if cs.value() < lightFarValue:
                walk(randomWalkTime, lightFarValue, False)

            while cs.value() >= lightFarValue:
                walk(0.2, lightFarValue, True)

            runWalk = False
            runSearch = True

        

        elif runSearch == True:

            debug_print("Search")

            search(lightFarValue)










if __name__ == '__main__':

    main()
