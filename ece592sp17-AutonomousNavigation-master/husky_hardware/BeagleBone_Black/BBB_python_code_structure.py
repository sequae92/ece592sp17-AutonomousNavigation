import horizon stuff
import pygame

import multiprocessing
import whatever else needed

#Short note on global variables and multithreading. Sharing variables is not ideal when using multithreading since
#if one thread tries to access it and another is changing it, itll cause problems.
# Thats why queues are used to coordinate stuff and not cause wierd
#behaviour in code. You are welcome to try implementing semaphores/mutexes and stuff if you
#how to use them. :D

#Also im using 3 different queues here as you will see.

def Joystick(Joy_queue,Auto_manual_queue,Emergency_queue):
    #intitiatte pygame or joystick and handle the button received from the
    #joystick and put them in a queue depending on press of the auto/manual switch button. For example
    #if the button down is the manual/auto switch buton then if this button is pressed, set
    # the flag and put stuff in queue which will be taken by Husky operator function..
    initaite joystick object usb port blah blah
    #Check if the rising or falling edge(when button is pressed) can be detected
    #since that is the better option
    while true:
        manual flag =False
        if (down button pressed)
            toggle (set/unset)  manual flag
            if manual flag set
                Auto_manual_queue.put("manual mode")
            else
                Auto_manual_queue.put("Auto mode")
        elif killswitch button pressed
            Emergency_queue.put(switcch off message)
        if manual flag set
            stop husky first by ising zero velocity
            set.differential_output(0,0,0,0)
            Joy_queue.put(the left analog stick axes button data)
            #label the data and put it or put in an array so that in husky function
            #we retrieve the first data as vertical and next as horizontal axis
        time.sleep(.1)

def Husky_opeartor(Joy_queue):

    intitiate global husky object usb port blah blah and open
    #look into starter.py in clearpath API
    #No need to subcribe to encoder data since that will be handled by JEtson
    #But data will need to be piped back to the jetson at all time to localize(this again depends
    #if they need encoder data but we will assume they do for now)
    #i.e when the manual mode is engaged encoder data is still sent to JEtson
    #
    while true:
        command=Joy_queue.get()
        if command not empty
            do some math considering the angular velocities
            set.differential_output(left wheel , right whee , left accel , right accel)
            # this is the one i used for control other day. Ill also put up the latest ROS husky
            #control script

def servo_killswitch(Emergency_queue):

    intitiate servo obect using adafriut or some library
    # Figure out the pulses for travel distance needed by the gear.
    while true:
        Switch_message= Emergency_queue.get()
        if message ==switch off
            activate servo to push out gear and return to original position

def Data_pipe(Auto_manual_queue):
     Set up the uart usb serial lines and whatever dependencies
     auto_manual_message== Auto mode
     bool Auto_mode ==True
     #start with auto mode.
     while True:
     auto_manual_message=Auto_manual_queue.get()
     if auto_manual_message== "Auto mode"
        Auto_mode ==True
    elif auto_manual_message=="manual mode"
        Auto_mode==False
    if (Auto_mode)
        keep just sending whatever is received from jetson and vice versa
    else
        send back whateer is received from the husky


if __name__ == "__main__":
    Joy_queue= mutliprocessing.Queue()
    Emergency_queue= multiprocessing.Queue()
    Auto_manual_queue= multiprocessing.Queue()
    Joystick_process= multiprocessing.Process(target=Joystick, args=(Joy_queue,Auto_manual_queue,Emergency_queue,))
    Husky_process= multiprocessing.Process(target= Husky_operator, args=(Joy_queue,))
    Servo_process= multiprocessing.Process(target=servo_killswitch,args=(Emergency_queue,))
    Data_pipe_process= multiprocessing.Process(target=Data_pipe, args=(Auto_manual_queue,))
    Joystick_process.start()
    Husky_process.start()
    Servo_process.start()
    Data_pipe_process.start()
