#! /usr/bin/python3

import control.core as core
import control.microcontroller as microcontroller
from control._def import *
import time
import os
import qtpy 

objectiveStore = core.ObjectiveStore(parent=None) # todo: add widget to select/save objective save
microcontroller = microcontroller.Microcontroller(version=CONTROLLER_VERSION)
navigationController = core.NavigationController(microcontroller, objectiveStore, None)

displacement  = 4

# reset the MCU
print('reset')
microcontroller.reset()
time.sleep(0.5)
# initialize the drivers
print('initialize')
microcontroller.initialize_drivers()
time.sleep(0.5)

print('configure')
microcontroller.configure_actuators()
time.sleep(0.5)

# navigationController.move_x(0.2)
# while microcontroller.is_busy():
#     time.sleep(0.005)

def disable_arms():
    microcontroller.set_axis_enable_disable(AXIS.X, 0) # enable y
    microcontroller.set_axis_enable_disable(AXIS.Y, 0) # disable x

def enable_arms():
    microcontroller.set_axis_enable_disable(AXIS.X, 1) # enable y
    microcontroller.set_axis_enable_disable(AXIS.Y, 1) # disable x

def home_right_arm():
    microcontroller.set_axis_enable_disable(AXIS.X, 1) # enable y
    microcontroller.set_axis_enable_disable(AXIS.Y, 0) # disable x

    print('homing y...')
    navigationController.home_y()
    t0 = time.time()
    while microcontroller.is_busy():
        time.sleep(0.005)
        if time.time() - t0 > 10:
            print('y homing timeout, the program will exit')
            sys.exit(1)
    navigationController.zero_y()

def home_left_arm():
    microcontroller.set_axis_enable_disable(AXIS.X, 0) # enable y
    microcontroller.set_axis_enable_disable(AXIS.Y, 1) # disable x

    print('homing x...')
    navigationController.home_x()
    t0 = time.time()
    while microcontroller.is_busy():
        time.sleep(0.005)
        if time.time() - t0 > 10:
            print('y homing timeout, the program will exit')
            sys.exit(1)
    navigationController.zero_x()

def move_right_arm(d):
    navigationController.move_y(d)
    while microcontroller.is_busy():
        time.sleep(0.005)

def move_left_arm(d):
    navigationController.move_x(d)
    while microcontroller.is_busy():
        time.sleep(0.005)

def home_arms():
    home_right_arm()
    move_right_arm(1)
    home_left_arm()
    move_left_arm(1)
    enable_arms()

    move_right_arm(-0.25)
    move_left_arm(-0.25)

def home_z():
    navigationController.home_z()
    while microcontroller.is_busy():
        time.sleep(0.005)
    navigationController.move_z(1)
    while microcontroller.is_busy():
        time.sleep(0.005)

def home():
    home_z()   
    home_arms()

home()

# quit application
microcontroller.close()
