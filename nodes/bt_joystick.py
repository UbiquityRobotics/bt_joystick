#!/usr/bin/python -u

#
# ROS node to interface a Magicsee R1 ring-held Bluetooth LE joystick/gamepad for /cmd_vel robot control
#
# The hand controller notifications are received by this node for the joystick and buttons.
#
# This node then publishes to either (or both) a /joy or /cmd_vel type of ROS topic
# The /joy topic gets joystick messages and the /cmd_vel topic gets 'twist' messages
# and all 6 buttons are mapped to the /joy 'buttons' array as 1 for pressed and 0 for released
# The default /cmd_vel topic and speeds may be changed using parameters
# Release of the joystick causes velocity in all modes to go back to 0
#
# Speed is increased with button A or decreased with button C (defined in g_speed_increase_bit and g_speed_decrease_bit)
# (speed_turbo button removed Oct 2018)
#
# Configuration is done in:  catkin_ws/src/demos/bt_joystick/bt_joystick.yaml
# The BT MAC address MUST match the hand controller to be used.
# Turn on the BT joystick and while it blinks blue led use 'hcitool lescan' for a Bluetooth Scan
# Alternately you can use other scan tools such as an App like nRFConnect
# find the address for 'Magicsee R1' (usually starts with FF:FF:80:06)
# Put this address in as the bt_mac_address in the bt_joystick.yaml file
#
# WARNING:  You MUST start the Bt Joystick first AND simul-press 'M' + 'B' before launching node!
#
# ROS Param            Default  Description
# bt_mac_address          xx    Bluetooth LE MAC Address 
# output_to_joy           1     Output control messages to the /joy ROS topic using Joy messages
# output_to_cmd_vel       0     Output control messages to the /cmd_vel ROS topic using twist messages
# speed_stopped           0.0   Robot X speed when stopped (can be used to null out robot offset)
# speed_fwd_normal        0.2   Robot forward X speed in M/Sec for joystick forward straight
# speed_fwd_inc           0.05  Increment in M/Sec for a speed increment
# speed_fwd_turning       0.2   Robot forward X speed in M/Sec for joystick in turn mode (not rotate mode)
# speed_rev_normal       -0.15  Robot reverse X speed in M/Sec for joystick reverse straight back
# angular_straight_rate   0.0   Robot Z angular rate when stopped (can be used to null out robot offset)
# angular_turning_rate    0.3   Robot Z angular rate in Rad/Sec when turning (not for rotate)
# angular_rotate_rate     0.4   Robot Z angular rate in Rad/Sec when rotating
# angular_rotate_inc      0.1   Increment in ad/Sec for a rotate increment
# angular_rotate_max      2.0   Max Robot Z angular rate in Rad/Sec

# Run the raw python code without full launch file
# Turn off then on the controller. Then press it's side 'M' at same time as 'B' and release
# Next run the node from catkin_ws:   catkin_ws> roslaunch bt_joystick bt_joystick.launch
# To run the node without ROS launch or the yaml config file you can run from where the script is:
# python nodes/bt_joystick.py FF:FF:80:06:6C:59
#
# System Limitations (beside having ROS environment and bt_joystick node configured)
# - Must start bluetooth.service with -compat and some config for a user in group bluetooth to use
# - User MUST hit M and B after EACH power-up of the controller to get to gamepad mode!
# - Must have the Bluetooth MAC address set in bt_joystick.yaml as  bt_mac_address rosparam
# - Must NOT have any device like Android paired to the Magicsee R1 or this node is shut out
#
# Development Info
# g_debug can enable ros logs or debug prints but do NOT ship in that state as it's flakey
#
# BUGS:
# - Must add ability to know we lost connection and issue 0.0 speed till re-connect
# - If we do loose connection must ignore first few notifications perhaps or we get a delayed command
# - If the joystick has paired it must be manually un-paired as this code does not do that yet
#    To unpair:  sudo bluetoothctl -a   then    disconnect FF:FF:80:06:6D:67 where the value is the MAC
#                You can check if paired in bluetoothctl using   'paired-devices', look for Magicsee R1
#

from __future__ import print_function

import sys
from threading import Event
from threading import Thread
from gattlib   import GATTRequester
import time

# import ROS related support including Twist messages
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# simple version string
g_version = "20181012"

# Bluetooth MAC address if none is supplied
g_bt_mac_address = "FF:FF:80:06:6C:59"

# This debug flag if set True enables prints and so on but cannot be used in production
g_debug = False

# Workaround for disconnect returning prior to truely being disconnected is an ugly delay
g_disconnectTime = float(0.4)

# Button and Joystick bytes
g_joystickBits = int(0)
g_buttonBits   = int(0)
g_fwdVel       = float(0.0)

# define bits we read from ring controller for keys pressed
button_a_bit   = int(4)   # 0x10
button_b_bit   = int(0)   # 0x01
button_c_bit   = int(3)   # 0x08
button_d_bit   = int(1)   # 0x02
button_o_bit   = int(6)   # 0x40
button_r_bit   = int(7)   # 0x80

# define what button bit is set in /joy message for each BT joystick button
button_a_joy_bit   = int(0)   # 0x10
button_b_joy_bit   = int(1)   # 0x01
button_c_joy_bit   = int(2)   # 0x08
button_d_joy_bit   = int(3)   # 0x02
button_o_joy_bit   = int(4)   # 0x40
button_r_joy_bit   = int(5)   # 0x80

# define which keys will be used to increment speed for max joystick values
g_speed_increase_bit = int(button_a_bit)
g_speed_decrease_bit = int(button_c_bit)

# Simple wrappers to prevent log overhead and use desired log functions for your system
def logAlways(message):
    rospy.loginfo(message)

def logDebug(message):
    global g_debug
    if g_debug == True:
        rospy.loginfo(message)


# Bluetooth Controller Notification Ability uses this class to setup notification requester
class Requester(GATTRequester):
    global g_debug
    def __init__(self, wakeup, *args):
        GATTRequester.__init__(self, *args)
        #print("Requester instantiated")
        self.wakeup = wakeup
        #print("Requester wakeup set")

    # The 'meat' of the BT notifications comes in here
    # We verify the proper packet type which has 5 bytes with first one as 0x1b
    # If packet is correct type we pull off the joystick bits and button bits for main logic
    def on_notification(self, handle, data):

        # The bits we see here are placed in globals below for main logic to decode
        global g_joystickBits     
        global g_buttonBits

        # print("- notification on handle: {}\n".format(handle))

        # Optionally show the data in hex printout but do NOT leave active for production
        if g_debug == True:
            for b in data:
                print(hex(ord(b)), end=' ')
            print("")

        # Walk the data and pick out joystick and button data.
        packetType = 0
        index = 0
        for b in data:
            if index == 0: 
                # get the type of notification packet
                packetType = int(ord(b))
            else:
                if packetType == int(0x1b):
                    if index == 3:
                        g_buttonBits = ord(b)
                    if index == 4:
                        g_joystickBits = ord(b)
            index = index + 1
        self.wakeup.set()


# Remain connected while receiving notifications
class ReceiveNotificationLooper(object):
    def __init__(self, bt_mac_address):
        self.received = Event()
        try:
            self.requester = Requester(self.received, bt_mac_address, False)
        except Exception:
            logAlways("Exception in ReceiveNotificationLooper Init!")
        try:
            self.connect()
            logDebug("Connected..")
            # This is where we differ from simple ReceiveNotification
            while not rospy.is_shutdown(): 
                if not self.requester.is_connected() == True:
                    logAlways("Connection lost! Re-connecting")
                    self.connect()
                    logAlways("Re-connected")
                logDebug("Receive a notification from BT Mac " + bt_mac_address)
                logDebug("Wait on notify.")
                self.wait_notification()
                logDebug("Notify done.")
            self.disconnect()
        except Exception:
            logAlways("Exception in ReceiveNotificationLooper!")
            time.sleep(0.3)


    def connect(self):
        if g_debug == True:
            logAlways("Flush stdout")
            sys.stdout.flush()
            # time.sleep(0.5)   # was 1 sec prior to Jan 16
            logAlways("Connecting...")
            # time.sleep(0.1)

        try:
            # if already connected connect() will throw runtime_error
            if self.requester.is_connected() == False:
                # remove True for wait so can run as user  self.requester.connect(True)
                self.requester.connect(True)
        except RuntimeError,e: 
            logAlways("Exception in connect: " + e.message)
        except Exception:
            logAlways("Exception in Connect!")
            time.sleep(0.2)

        logDebug("OK!")     #time.sleep(0.5)   # was 1 sec prior to Jan 16

    def disconnect(self):
        logDebug("Disconnecting...")

        try:
            self.requester.disconnect()
            
            # it seems we are not really disconnected and must wait for that state
            # Checking is_connected() in this loop does NOT fix the issue
            #dcRetries = int(0)
            #while self.requester.is_connected() == True:
            #    dcRetries = dcRetries + 1
            #    if dcRetries > 10:
            #        logAlways("disconnect timeout!")
            #        break
            #    time.sleep(0.1)
                    
            time.sleep(g_disconnectTime)

        except Exception:
            logAlways("Exception in disconnect!")
            time.sleep(0.3)

        if g_debug == True:
            logAlways("OK!")
            time.sleep(1)

    def wait_notification(self):
        logDebug("\nMake your device send a  notification ...")

        self.received.wait()


def btjoystickcapture():
    global g_bt_mac_address
    global g_debug

    while not rospy.is_shutdown(): 
        logDebug("Receive a notification from BT Mac " + g_bt_mac_address)

        # choose 'Looper' to remain connected or just ReceiveNotification for connect each time
        ReceiveNotificationLooper(g_bt_mac_address )  

        logDebug ("Notification done")

# test a single bit of an integer to see if it is set or clear
def testBit(int_type, offset):
    mask = 1 << offset
    return(int_type & mask)

class btjoystick():
    def __init__(self):
        
        # The bits we see here are placed in globals by notification callback
        global g_bt_mac_address
        global g_joystickBits
        global g_buttonBits

        g_joystickBits  = 1
        g_buttonBits    = 1
        oldJoystickBits = -1
        oldButtonBits   = -1

        logAlways("Starting joystick thread that publishes twist to ROS")
    
        # initiliaze
        rospy.init_node('bt_joystick', anonymous=False)

        # The bit codes for the joystick movement
        # User MUST hit M and B to get to gamepad mode!
        joyFwdBits          = int('0x10',16)
        joyFwdRightBits     = int('0x20',16)
        joyRotRightBits     = int('0x60',16)
        joyFwdLeftBits      = int('0x00',16)
        joyRotLeftBits      = int('0x40',16)
        joyRevBits          = int('0x90',16)

        logAlways("Setup Node Parameters ...");
        # Now adjust the parameters from ros config params if they exist
        # Comment this out to not worry about config but if used btjoystick.cfg will set the defaults
        self.bt_mac_address = rospy.get_param("~bt_mac_address", g_bt_mac_address)
        g_bt_mac_address = self.bt_mac_address

        # Define which sort of outputs this node will send to ROS topics for bot control
        self.output_to_joy = rospy.get_param("~output_to_joy", 1)
        self.output_to_cmd_vel = rospy.get_param("~output_to_cmd_vel", 0)

        # Best to keep fwd_max / fwd_inc integer and equal to rev_max / rev_inc
        self.speed_fwd_max = rospy.get_param("~speed_fwd_max", 2.0)
        self.speed_fwd_inc = rospy.get_param("~speed_fwd_inc", 0.1)
        self.speed_rev_max = rospy.get_param("~speed_fwd_max", -1.0)
        self.speed_rev_inc = rospy.get_param("~speed_fwd_inc", 0.05)
        self.speed_stopped = rospy.get_param("~speed_stopped", 0.0)
        self.speed_fwd_normal = rospy.get_param("~speed_fwd_normal", 0.3)
        self.speed_fwd_turning = rospy.get_param("~speed_fwd_turning", 0.2)
        self.speed_rev_normal = rospy.get_param("~speed_rev_normal", -0.15)
        self.speed_rev_inc = rospy.get_param("~speed_fwd_inc", 0.05)
        self.angular_straight_rate = rospy.get_param("~angular_straight_rate", 0.0)
        self.angular_turning_rate = rospy.get_param("~angular_turning_rate", 0.3)
        self.angular_rotate_rate = rospy.get_param("~angular_rotate_rate", 1.5)
        self.angular_rotate_inc = rospy.get_param("~angular_rotate_inc", 0.1)
        self.angular_rotate_max = rospy.get_param("~angular_rotate_inc", 3.0)

        logAlways("Node Parameters:")
        logAlways("MAC: " + self.bt_mac_address)
        logAlways("Fwd: " + str(self.speed_fwd_normal) + " FwdTurning: " + str(self.speed_fwd_turning) + " TurboMult: ")
        logAlways("AngTurningRadSec: " + str(self.angular_turning_rate) + " AngRotateRadSec: " + str(self.angular_rotate_rate))
        if self.output_to_cmd_vel == 1:
            logAlways("ROS twist messages will appear on ROS topic: /cmd_vel_joy")
        if self.output_to_joy == 1:
            logAlways("ROS joystick messages will appear on ROS topic: /joy")

        #  tell user how to stop TurtleBot
        logAlways("To stop the BT Joystick node use CTRL + Z")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # Setup ROS publisher(s) depending on what outputs you have enabled for this node
        # Setup the joy message if joy ROS output is to be used
        joy_msg = Joy()

        #if self.output_to_joy == 1:
        #    # Create a publisher which can "talk" to the ROS /joy topic and act like a joystick
        #    self.joy = rospy.Publisher('/joy', Joy, queue_size=10)
        self.joy = rospy.Publisher('/joy', Joy, queue_size=10)

        # Setup the Twist message if cmd_vel ROS output is to be used
        move_cmd = Twist()
        #if self.output_to_cmd_vel == 1:
        #    #  Create a publisher which can "talk" to TurtleBot and tell it to move
        #    # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        #    self.cmd_vel = rospy.Publisher('/cmd_vel_joy', Twist, queue_size=10)
        self.cmd_vel = rospy.Publisher('/cmd_vel_joy', Twist, queue_size=10)
        
        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        
        #  let's set velocity to 0 at this time
        move_cmd.linear.x = self.speed_stopped
        #  let's turn at 0 radians/s
        move_cmd.angular.z = self.speed_stopped

        joy_msg.buttons = [int(0), int(0), int(0), int(0), int(0), int(0)]
        joy_msg.axes = [0.0, 0.0]

        # fire off the thread that accepts bluetooth controller notifications
        logAlways("Fire off joystick event capture thread for MAC addr " + self.bt_mac_address)
        Thread(target = btjoystickcapture).start()
        logAlways("BT Joystick notification thread running")

        if rospy.is_shutdown():
            logAlways("ROS appears to be shutdown. Run roscore")

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # We do not force linear and angular speeds to 0 unless joystick is not pressed
            # move_cmd.linear.x = 0.0
            # move_cmd.angular.z = 0.0
            # Show when the bits have changed
            if g_buttonBits != oldButtonBits:
                logDebug("Button Bits Changed from " + str(oldButtonBits) + "  to " + str(g_buttonBits))
                joy_msg.buttons = [int(0), int(0), int(0), int(0), int(0), int(0)]

                # update the button bits for joy message
                if (testBit(g_buttonBits, button_a_bit) != 0):
                    logAlways("Set Button Bit for button A")
                    joy_msg.buttons[button_a_joy_bit] = 1
                if (testBit(g_buttonBits, button_b_bit) != 0):
                    logAlways("Set Button Bit for button B")
                    joy_msg.buttons[button_b_joy_bit] = 1
                if (testBit(g_buttonBits, button_c_bit) != 0):
                    logAlways("Set Button Bit for button C")
                    joy_msg.buttons[button_c_joy_bit] = 1
                if (testBit(g_buttonBits, button_d_bit) != 0):
                    logAlways("Set Button Bit for button D")
                    joy_msg.buttons[button_d_joy_bit] = 1
                if (testBit(g_buttonBits, button_o_bit) != 0):
                    logAlways("Set Button Bit for O")
                    joy_msg.buttons[button_o_joy_bit] = 1
                if (testBit(g_buttonBits, button_r_bit) != 0):
                    logAlways("Set Button Bit for R")
                    joy_msg.buttons[button_r_joy_bit] = 1


                # Here we do forward/reverse speed increase and decreases based on buttons A and C.
                # We cap out at max limits for increase and stop at 0 for decreases.
                # look for speed increase command
                if ( not testBit(oldButtonBits, g_speed_increase_bit)) and testBit(g_buttonBits, g_speed_increase_bit):
                    # adjust forward speed faster
                    self.speed_fwd_normal += self.speed_fwd_inc
                    if self.speed_fwd_normal > self.speed_fwd_max:
                        self.speed_fwd_normal = self.speed_fwd_max
                    # adjust reverse speed faster
                    self.speed_rev_normal -= self.speed_rev_inc
                    if self.speed_rev_normal < self.speed_rev_max:
                        self.speed_rev_normal = self.speed_rev_max
                    logAlways("Increase fwd speed to " + str(self.speed_fwd_normal) + " and rev speed to " + str(self.speed_rev_normal))
                # look for speed decrease command
                if ( not testBit(oldButtonBits, g_speed_decrease_bit)) and testBit(g_buttonBits, g_speed_decrease_bit):
                    # adjust forward speed lower
                    self.speed_fwd_normal -= self.speed_fwd_inc
                    if self.speed_fwd_normal < 0.0:
                        self.speed_fwd_normal = 0.0
                    # adjust reverse speed lower
                    self.speed_rev_normal += self.speed_rev_inc
                    if self.speed_rev_normal > 0.0:
                        self.speed_rev_normal = 0.0
                    logAlways("Decrease fwd speed to " + str(self.speed_fwd_normal) + " and rev speed to " + str(self.speed_rev_normal))

            # the joystick notification thread maintains bits for joystick and buttons
            logDebug("Switch on Joystick bits")
            try:
                if g_joystickBits == 16:  # joyFwdBits:
                    logDebug("FORWARD")
                    move_cmd.linear.x  = self.speed_fwd_normal
                    move_cmd.angular.z = self.angular_straight_rate
                elif g_joystickBits == 32:  # joyFwdRightBits:
                    logDebug("FWDRIGHT")
                    move_cmd.linear.x  = self.speed_fwd_turning
                    move_cmd.angular.z = self.angular_turning_rate * -1.0
                elif g_joystickBits == 96:  # joyRotRtBits:
                    logDebug("ROTRIGHT")
                    move_cmd.linear.x  = self.speed_stopped
                    move_cmd.angular.z = self.angular_rotate_rate * -1.0
                elif g_joystickBits == 0:   # joyFwdLeftBits:
                    logDebug("FWDLEFT")
                    move_cmd.linear.x  = self.speed_fwd_turning
                    move_cmd.angular.z = self.angular_turning_rate
                elif g_joystickBits ==  64: # joyRotLeftBits:
                    logDebug("ROTLEFT")
                    move_cmd.linear.x  = self.speed_stopped
                    move_cmd.angular.z = self.angular_rotate_rate
                elif g_joystickBits == 144:   # joyRevBits:
                    logDebug("REVERSE")
                    move_cmd.linear.x  = self.speed_rev_normal
                    move_cmd.angular.z = self.angular_straight_rate * -1.0;
                elif g_joystickBits == 80:   # no buttons or joystick pressed
                    # We do force linear and angular speeds to 0 when joystick is not pressed
                    logDebug("Joystick idle. STOP")
                    move_cmd.linear.x  = self.speed_stopped
                    move_cmd.angular.z = self.angular_straight_rate;
                else:
                    # We do force linear and angular speeds to 0 when joystick is not pressed
                    logDebug("Unrecognized state so ignore it")
            except Exception:
                logAlways("bt_joystick bad bit switch!")

            logDebug("DONE Switch on Joystick bits of " + str(g_joystickBits))

            # speed_turbo mode removed Oct 2018  It was forUser hits the return (top) front button for faster speed
            #if g_buttonBits == 128:
            #    move_cmd.linear.x = move_cmd.linear.x * self.speed_turbo_multiplier

            if g_debug == True:
                if g_joystickBits != oldJoystickBits or g_buttonBits != oldButtonBits:
                    logAlways("New JoyBits for speed " + move_cmd.linear.x + " turnRate " + move_cmd.angular.z)
            oldJoystickBits = g_joystickBits
            oldButtonBits   = g_buttonBits

             # publish the joystick output messages
            if self.output_to_joy == 1:
                # convert linear and angular speeds into joystick values from 0-1 based on max rates
                joy_linear  = move_cmd.linear.x / self.speed_fwd_max
                joy_angular = move_cmd.angular.z / self.angular_rotate_max
                logDebug("Publish Joystick speeds: angular " + str(joy_angular) + " linear: " + str(joy_linear))
                joy_msg.axes = [joy_angular, joy_linear]

                # pack rhe button bits
                self.joy.publish(joy_msg)

            # publish the velocity
            if self.output_to_cmd_vel == 1:
                self.cmd_vel.publish(move_cmd)

            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
        logAlways("ROS HAS SHUTDOWN!")

    def cfg_callback(self, config, level):
        self.config = config
        return config

    def shutdown(self):
        # stop turtlebot
        logAlways("SHUTDOWN! Stopping this node")
        sys.exit(1)
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    print("Running with version: " + g_version)
 
    try:
        btjoystick()
    except RuntimeError,e: 
        logAlways("Exception in bt_joystick: " + e.message)
    except Exception:
        logAlways("bt_joystick node terminated.")
