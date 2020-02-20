#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
#joystick
import socket
import struct
#interface
import curses
import os
import threading 
import numpy


#******************************************************#
#How to use this script:
#  Every main loop, joystick position is read.
#       pitch position is stored in: js.pos[0]
#       roll position is stored in:  js.pos[1]
#  Every main loop, joystick forces are updated in 'calc_controller_output'
#       pitch force is set to: js.force[0]
#       roll force is set to:  js.force[1]
#  Vehicle outputs are caught by a message handler whenever the right MAVLINK message is sent.
#       NAV_CONTROLLER_OUTPUT yields: js.cross_track_error
#       NAMED_VALUE_FLOAT{name:js_out_*} stores values in: js.vehicle_output[*-1]
#  Updated outputs are sent to the vehicle mimicking changes in the RC channels 
#       js.controller_output[*] -> js_<*+1> in the Ardupilot code
#
#
#  All input and output of values is handled in functions in:
#        /ardupilot/APMRover2/mode.cpp
#            Mode::apply_human_control( ... )     //decodes input from joystick by checking RC channels
#            Mode::get_pilot_joystick( ... )      //sends MAVLINK messages with outputs for this script
#******************************************************#


# comms settings
vehicle_host_port = '127.0.0.1:14551'
loop_delay = 0.1
joystick_host = '127.0.0.1'  # IP address of machine running CLS2Sim
joystick_port = 15090        # UDP port configured in CLS2Sim
joystick_timeout = 8.0

# RC channel to Ardupilot JS input map:
js_1 = '4'
js_2 = '5'
js_3 = '6'
js_4 = '7'

# mission plan settings
number_of_lanes = 10
magic_lane_number = number_of_lanes * 2


class joystick_controller:

    # joystick interaction settings
    threshold = 0.45
    maxForce = 1000

    # initialization
    joystick_center = 0.5
    output_center = 1500
    output_scaling = 250

    joystick_input = joystick_center
    vehicle_output = [0,0,0,0,0,0,0,0]
    controller_output = [0,0,0,0]
    
    force = [0,0,0,0]
    pos = [0,0,0,0]

    lane_change_flag = 0
    trigger_lane_change = 0
    cross_track_error = 0
    actuators = {}
    name = {}

    roll_offset_gain = 1000
    pitch_offset_gain = 1000
    vehicle_output_gain = [1, 1, 0.05, 1, 1, 1, -0.5, 1]
    cross_track_gain = 0
    
    def calc_controller_output(self):

        # React to roll position:
        pitch_offset = 2.0*self.pos[0]-1.0 - self.vehicle_output[6]*self.vehicle_output_gain[6]
        roll_offset = 2.0*self.pos[1]-1.0 - self.vehicle_output[2]*self.vehicle_output_gain[2]

        # Calculate joystick forces based on positions and vehicle outputs:
        self.force[0] = 0
        self.force[0] = pitch_offset*self.pitch_offset_gain
        self.force[1] = roll_offset*self.roll_offset_gain #-self.cross_track_error*self.cross_track_gain + 
        pitch_force = self.force[0]
        roll_force = self.force[1]

        # Return desired outputs to vehicle:
        self.controller_output[0] =  self.output_scaling*roll_offset + self.output_center
        self.controller_output[1] =  -self.output_scaling*pitch_offset + self.output_center # pitch is reversed!
        self.controller_output[2] =  roll_force + self.output_center
        self.controller_output[3] =  pitch_force + self.output_center
        
        # Lane change threshold value reached (if not already over).
        if (self.lane_change_flag==0):
            if (roll_offset > self.threshold):
                self.lane_change_flag = 1
                self.trigger_lane_change = 1
            else:
                if (roll_offset < -self.threshold):
                    self.lane_change_flag = 1
                    self.trigger_lane_change = -1
                    
        # Check to see if we have come back down from threshold (enabling another change).
        else:
            if (abs(roll_offset) < self.threshold):
                self.lane_change_flag = 0

    
def lane_change(vehicle,magic_lane_number,direction):  # react to lane change command
    nextwaypoint=vehicle.commands.next
    if direction>0:
        if nextwaypoint < magic_lane_number:
            skiptowaypoint = nextwaypoint+magic_lane_number+2
        else:
            skiptowaypoint = nextwaypoint-magic_lane_number+2
    else:
        if nextwaypoint < magic_lane_number:
            skiptowaypoint = nextwaypoint+magic_lane_number-2
        else:
            skiptowaypoint = nextwaypoint-magic_lane_number-2    
    print('Next Way Point: %s' % nextwaypoint)
    print('Skipping to Way Point %s.' % skiptowaypoint)
    vehicle.commands.next = skiptowaypoint
    print('Next Way Point: %s' % skiptowaypoint)
    
def main(win):
    # Initialize curses window:
    win.nodelay(True)
    win.clear()                
    win.addstr("Waiting for First Input!")

    js = joystick_controller()
    vehicle_output = js.output_center
    
    # Create a UDP socket for joystick communication.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 5) # TTL=5 to jump over routers
    sock.settimeout(joystick_timeout)
    
    # Connect to the vehicle.
    print("Connecting to vehicle on: %s" % (vehicle_host_port,))
    vehicle = connect(vehicle_host_port, wait_ready=False)
    print("Connected to Vehicle")
                     
    # Setup listener for navigation message:
    @vehicle.on_message('NAV_CONTROLLER_OUTPUT')
    def listener(self, name, message):
        js.cross_track_error = float(message.xtrack_error)
        js.speed_error = float(message.aspd_error)
        js.nav_bearing = float(message.nav_bearing)
        js.target_bearing = float(message.target_bearing)


    # Setup listener for navigation message:
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def listener(self, name, message):
        output_name = message.name

    # Setup listener for navigation message:
    @vehicle.on_message('NAMED_VALUE_FLOAT')
    def listener(self, name, message):
        output_name = message.name
        if (output_name[0:7]=='js_out_'):
            output_number = int(output_name[7])
            js.vehicle_output[output_number-1] = float(message.value)

    debug_value = 0.1
    joystick_attempts = 1
    js.pos = [js.joystick_center+debug_value,js.joystick_center,js.joystick_center,js.joystick_center]
    
    while 1:
     #   try:
        if 1:
            time.sleep(loop_delay)

            if joystick_attempts>0:
                try:
                    # Get joystick state******************************#:
                    # Unsigned integer (command) and 4 signed integers (force data)
                    # Send and receive:
                    request = struct.pack('<Iiiii', 0xAF, js.force[0], js.force[1], js.force[2], js.force[3])
                    sock.sendto(request, (joystick_host, joystick_port))
                    response, address = sock.recvfrom(20)
                    # Read one unsigned integer(command) and 4 floats (position data)
                    result, js.pos[0], js.pos[1], js.pos[2], js.pos[3] = struct.unpack('<Iffff', response)
                    #********************************************#
                except Exception as e:
                    print('Joystick Comm Failure: Values May Be Set Locally')
                    joystick_attempts += -1
            else:
                js.pos = [js.joystick_center+debug_value,js.joystick_center,js.joystick_center,js.joystick_center]
                
            #**************************************************#
            # Apply controller to vehicle output:
            js.calc_controller_output()
            #**************************************************#
            
            vehicle.channels.overrides[js_1] = js.controller_output[0]
            vehicle.channels.overrides[js_2] = js.controller_output[1]
            vehicle.channels.overrides[js_3] = js.controller_output[2]
            vehicle.channels.overrides[js_4] = js.controller_output[3]
            
            # Print positions
            win.clear()
            win.addstr('Joystick:' + str(numpy.around(numpy.array(js.pos[0:4]),2)))
            win.addstr('  Control:' + str(numpy.around(numpy.array(js.controller_output),0)))
            win.addstr('  Vehicle:' + str(numpy.around(numpy.array(js.vehicle_output),1)))

            # Exectute lane change if triggered:
            if js.lane_change_flag :
                win.addstr("  LANE CHANGE COMMANDED: Center Stick to Regain Control")
            
            if (js.trigger_lane_change==0):
                 pass
            else:
                lane_change(vehicle,magic_lane_number,js.trigger_lane_change)
                js.trigger_lane_change = 0
                
            # check for keyboard input, quit if 'enter' key hit:
            try:
                key = win.getkey()
                if key == 'KEY_UP':
                    debug_value += 0.05
                    print('Key Up')
                if key == 'KEY_DOWN':
                    debug_value += -0.05
                if key == os.linesep:
                    #Clear overrides and close vehicle object before exiting script
                    vehicle.channels.overrides = {}
                    print("Close vehicle object")
                    vehicle.close()
                    break
            except Exception as e:
                # No input
                # resend joystick input to keep it from expiring
                pass
            
curses.wrapper(main)


