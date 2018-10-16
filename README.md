# Bluetooth BLE Joystick node for the Magicsee R1 joystick in gamepad mode

This code implemente a ROS node to interface a Magicsee R1 ring-held Bluetooth LE in gamepad mode

Default as of Oct 2018 is to output joystick messages to ROS topic of /joy
It is still possible to output twist messages to the ROS topic /cmd_vel_joy but not by default any longer

The hand controller notifications are received by this node for the joystick and buttons.
This node then publishes to ROS topic with 'twist' messages normally we use topic /cmd_vel
The default /cmd_vel topic and speeds may be changed using parameters

Forward and reverse speed are increased with button A or decreased with button C (works while holding joystick)
Speed goes to 0 of joystick is released.  The joystick is 'binary' at this time so is not able to output 'analog' type values yet from joystick fine position changes.

Configuration is done in:  catkin_ws/src/demos/bt_joystick/bt_joystick.yaml
The BT MAC address MUST match the hand controller to be used.
Use App like nRFConnect and get address for 'Magicsee R1' (usually starts with FF:FF:80:06)

WARNING:  You MUST start the Bt Joystick first AND simul-press 'M' + 'B' for gamepad mode before launching node!
There are some forms of recovery but not all forms of joystick off to on can be detected just yet.

## Dependencies

There are many system level configuration issues that must be present for this node to operate.
In summary the python gattlib library is used and there is other setup and bluetooth support for the Raspberry Pi 2 or 3 depending on what is being used must be present in the kernel.   The node can run as root bot for a user to run this node requires some tricky permissions things to be setup for the user and for bluetooth mode itself.  We hope to do this someday but it is not all there as of Oct 2018.
This README does not yet explain the setup as this node is to be included on the Ubiquity Robotics platforms that use a Raspberry Pi 3 with the kernel and system configuration required to run this node.

At a minimum, if your image has a great many things the following commands are necessary:

```
sudo apt install libbluetooth-dev
pip install gattlib
```

For those who are curious as to what it takes all together on a base Ubuntu image here are details.  To limit the number of lines below I use semicolin to let me show multiple commands on one line below. 

```
sudo apt-get install -y bluetooth bluez bluez-tools bluez-firmware blueman libbluetooth-dev
sudo apt-get install pkg-config libboost-python-dev libboost-thread-dev libbluetooth-dev libglib2.0-dev python-dev
sudo apt-get -y install python-pip;   pip install --upgrade setuptools;
Suggest you have a swapfile setup and if not look into:  sudo apt-get install dphys-swapfile
export MAKEFLAGS="-j 2" ; sudo pip install gattlib      (MAKEFLAGS export and ulimit to not soak up all memory)
```

## Raspberry Pi 3B+
The June 2018 Ubiquity kernel images are not yet setup to have the proper driver for bluetooth on the 2018 Pi 3B+ as they use a different chipset.  Ubiquity plans on getting a Kernel image that does have the proper driver soon as a high priority but we have many 'high priorities' so we don't have a precise date.  In that image we hope to have the complex installs and so on all on the image as well as proper driver for the bluetooth.


## Configuration

This node must have as a minimum the ROS parameter setup in bt_joystick.yaml for the controller in use.
You may run this command on a configured system to find the Magicsee R1 MAC address then use it.

    hcitool lescan               (After you find the Magicsee you may use Control-C)

Here is an explanation of the parameters in bt_joystick.yaml

    ROS Param            Default  Description
    bt_mac_address  FF:FF:80:06:6C:59     Bluetooth LE MAC Address (EACH unit is different MAC Addr)
    output_to_joy           1     Output ROS joystick messages to the /joy ROS topic
    output_to_cmd_vel       0     Output ROS twist messages to the /cmd_vel_joy ROS topic
    speed_stopped           0.0   Robot X speed when stopped (can be used to null out robot offset)
    speed_fwd_normal        0.2   Robot forward X speed in M/Sec for joystick forward straight
    speed_fwd_turning       0.2   Robot forward X speed in M/Sec for joystick in turn mode (not rotate mode)
    speed_rev_normal       -0.15  Robot reverse X speed in M/Sec for joystick reverse straight back
    speed_turbo_multiplier  1.75  Speed multiplier for X speed if in a mode where X is non-zero
    angular_straight_rate   0.0   Robot Z angular rate when stopped (can be used to null out robot offset)
    angular_turning_rate    0.3   Robot Z angular rate in Rad/Sec when turning (not for rotate)
    angular_rotate_rate     0.4   Robot Z angular rate in Rad/Sec when rotating

## Manually running the bt_joystick node

You may manually start this python ROS node as root as long as 'roscore' is running already and the ROOT window you are using has been setup to know ROS environment using this set of commands if in the bt_joystick local repository

```
hcitool lescan             (find mac addr for Magicsee to use for the script then use Ctrl-C)
source /opt/ros/kinetic/setup.bash
python nodes/bt_joystick.py FF:FF:80:06:6C:59
```

## System Limitations

Beside having ROS environment and bt_joystick node configured,
there are known design limitation or key operational thing to be known as follows

    - We do not have a formula for running this node as non-root but are looking into that ability
    - Must start bluetooth.service with -compat and some config for a user in group bluetooth to use
    - User MUST hit M and B after EACH power-up of the controller to get to gamepad mode!
    - Must have the Bluetooth MAC address set in bt_joystick.yaml as  bt_mac_address rosparam
    - Must NOT have any device like Android paired to the Magicsee R1 or this node is shut out

