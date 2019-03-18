#!/usr/bin/env python3

'''COM2009-3009 EV3DEV TEST PROGRAM'''



# Connect left motor to Output C and right motor to Output B

# Connect an ultrasonic sensor to Input 3



import os

import sys

import time

import ev3dev.ev3 as ev3



# state constants

ON = True

OFF = False





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








#Part 1.2
"""def main():

    # set the console just how we want it

    reset_console()

    set_cursor(OFF)

    set_font('Lat15-Terminus24x12')



    # set the ultrasonic sensor variable

    us3 = ev3.UltrasonicSensor('in3')



    oldM = 0

    newM = 0

    oldS = 0.0

    newS = 0.0

    count = 0

    minDs = 0

    maxDs = 0



    for i in range (0,1000):

        ds = us3.value()

        time.sleep(0.01)



        # display the distance on the screen of the device

        print('Distance =',ds)



        # print the distance to the output panel in VS Code

        debug_print('Distance =',ds)



        count+=1

        if count == 1:

            oldM = ds

            newM = ds

            minDs = ds

            maxDs = ds

            oldS = 0.0

        else:

            newM = oldM + (ds - oldM)/count

            newS = oldS + (ds - oldM)*(ds-newM)

            oldM = newM

            oldS = newS

            if ds < minDs:

                minDs = ds

            if ds > maxDs:

                maxDs = ds

        

        debug_print ('Mean =',newM)

        if count >1:

            debug_print ('Variance =',newS/(count-1))



    debug_print ('Min =',minDs)

    debug_print ('Max =',maxDs)



    # announce program end

    ev3.Sound.speak('Test program ending').wait()
"""

#Part 1.3
"""def main():
    # set the motor variables
    mb = ev3.LargeMotor('outB')
    mc = ev3.LargeMotor('outC')
    sp = -25
    ps = 25

    while (True):
        # move
        mb.run_direct(duty_cycle_sp=sp)
        mc.run_direct(duty_cycle_sp=sp)
        time.sleep(5)

        # stop
        mb.run_direct(duty_cycle_sp=0)
        mc.run_direct(duty_cycle_sp=0)

        mb.run_direct(duty_cycle_sp=sp)
        mc.run_direct(duty_cycle_sp=ps)
        time.sleep(2)
"""


#Part 1.5
#
def main():
    us3 = ev3.UltrasonicSensor('in3')
    us2 = ev3.UltrasonicSensor('in2')
    sp = -10
    mb = ev3.LargeMotor('outB')
    mc = ev3.LargeMotor('outC')
    r = 0
    b = 0
    e = r - b
    u = 0
    ic = time.clock() #initial clock time
    dt = 0
    de = 0
    t = 0
    chillMode = 0.08
    TU = 0.55
    KU = 0.06
    KP = 0.6 * KU
    KI = (1.2 * KU) / TU
    KI = KI * dt
    KD = (3 * KU * TU) / 40
    dt = time.clock() - t
    KD = KD/dt
    maxMotorValue = 100
    t_interval = ic
    integral = 0
    m = (r*2)/maxMotorValue
    bMotorValue = 0
    cMotorValue = 0
    turn = 0
    speed = 70
    while (True):
        t = time.clock() - ic
        #Change the value of the set distance
        """if (t - t_interval >= 1):
            if (r == 500):
                #r = 300
                debug_print("r is now 300")
                KP += 0.5
            else:
                #r = 500
                debug_print("r is now 500")
                KP += 0.5
            t_interval = t"""
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
        mb.run_direct(duty_cycle_sp=int(bMotorValue))
        mc.run_direct(duty_cycle_sp=int(cMotorValue))

        e = e1
        b = (us3.value()*-1) + us2.value()



if __name__ == '__main__':

    main()
