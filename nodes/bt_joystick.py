#!/usr/bin/python -u

#
# ROS node to interface a Magicsee R1 ring-held Bluetooth LE joystick/gamepad for /cmd_vel robot control
#
# The hand controller notifications are received by this node for the joystick and buttons.
# This node then publishes to ROS topic with 'twist' messages normally we use topic /cmd_vel
# The default /cmd_vel topic and speeds may be changed using parameters 
#
# Configuration is done in:  catkin_ws/src/demos/bt_joystick/bt_joystick.yaml
# The BT MAC address MUST match the hand controller to be used.
# Use App like nRFConnect and get address for 'Magicsee R1' (usually starts with FF:FF:80:06)
#
# WARNING:  You MUST start the Bt Joystick first AND simul-press 'M' + 'B' before launching node!
#
# ROS Param            Default  Description
# bt_mac_address          xx    Bluetooth LE MAC Address 
# speed_stopped           0.0   Robot X speed when stopped (can be used to null out robot offset)
# speed_fwd_normal        0.2   Robot forward X speed in M/Sec for joystick forward straight
# speed_fwd_turning       0.2   Robot forward X speed in M/Sec for joystick in turn mode (not rotate mode)
# speed_rev_normal       -0.15  Robot reverse X speed in M/Sec for joystick reverse straight back
# speed_turbo_multiplier  1.75  Speed multiplier for X speed if in a mode where X is non-zero
# angular_straight_rate   0.0   Robot Z angular rate when stopped (can be used to null out robot offset)
# angular_turning_rate    0.3   Robot Z angular rate in Rad/Sec when turning (not for rotate)
# angular_rotate_rate     0.4   Robot Z angular rate in Rad/Sec when rotating

# Run the raw python code without full launch file
# Turn off then on the controller. Then press it's side 'M' at same time as 'B' and release
# Next run the node from catkin_ws:   catkin_ws> roslaunch bt_joystick bt_joystick.launch
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

# simple version string
g_version = "20180207"

# Bluetooth MAC address if none is supplied
g_bt_mac_address = "FF:FF:80:06:6D:67"

# This debug flag if set True enables prints and so on but cannot be used in production
g_debug = False

# Workaround for disconnect returning prior to truely being disconnected is an ugly delay
g_disconnectTime = float(0.4)

# Button and Joystick bytes
g_joystickBits = int(0)
g_buttonBits   = int(0)
g_fwdVel       = float(0.0)

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

        self.speed_stopped = rospy.get_param("~speed_stopped", 0.0)
        self.speed_fwd_normal = rospy.get_param("~speed_fwd_normal", 0.2)
        self.speed_fwd_turning = rospy.get_param("~speed_fwd_turning", 0.2)
        self.speed_rev_normal = rospy.get_param("~speed_rev_normal", -0.15)
        self.speed_turbo_multiplier = rospy.get_param("~speed_turbo_multiplier", 1.75)
        self.angular_straight_rate = rospy.get_param("~angular_straight_rate", 0.0)
        self.angular_turning_rate = rospy.get_param("~angular_turning_rate", 0.3)
        self.angular_rotate_rate = rospy.get_param("~angular_rotate_rate", 1.5)


        logAlways("Node Parameters:")
        logAlways("MAC: " + self.bt_mac_address)
        logAlways("ROS cmd_vel topic: /cmd_vel_base")
        logAlways("Fwd: " + str(self.speed_fwd_normal) + " FwdTurning: " + str(self.speed_fwd_turning) + " TurboMult: " + str(self.speed_turbo_multiplier))
        logAlways("AngTurningRadSec: " + str(self.angular_turning_rate) + " AngRotateRadSec: " + str(self.angular_rotate_rate))

        #  tell user how to stop TurtleBot
        logAlways("To stop the BT Joystick node use CTRL + Z")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        #  Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel_base', Twist, queue_size=10)
        
        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        
        # Twist is a datatype for velocity
        move_cmd = Twist()

        #  let's set velocity to 0 at this time
        logAlways("Checkpoint 50")
        move_cmd.linear.x = self.speed_stopped
        #  let's turn at 0 radians/s
        move_cmd.angular.z = self.speed_stopped
        
        # fire off the thread that accepts bluetooth controller notifications
        logAlways("Fire off joystick event capture thread for MAC addr " + self.bt_mac_address)
        Thread(target = btjoystickcapture).start()
        logAlways("BT Joystick notification thread running")

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            if g_debug == True:
                # Show when the bits have changed
                if g_joystickBits != oldJoystickBits:
                    logDebug("Joystick Bits Changed to: " + str(g_joystickBits) + " buttonBits: " + str(g_buttonBits))
            # the joystick notification thread maintains bits for joystick and buttons
            # logDebug("Switch on Joystick bits")
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
                else:
                    logDebug("Controller IDLE ")
                    move_cmd.linear.x  = self.speed_stopped
                    move_cmd.angular.z = self.angular_straight_rate;
            except Exception:
                logAlways("bt_joystick bad bit switch!")
            # logDebug("DONE Switch on Joystick bits")
            # User hits the return (top) front button for faster speed
            if g_buttonBits == 128:
                move_cmd.linear.x = move_cmd.linear.x * self.speed_turbo_multiplier
            if g_debug == True:
                if g_joystickBits != oldJoystickBits or g_buttonBits != oldButtonBits:
                    logAlways("New JoyBits for speed " + move_cmd.linear.x + " turnRate " + move_cmd.angular.z)
            oldJoystickBits = g_joystickBits
            oldButtonBits   = g_buttonBits
            # publish the velocity
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
