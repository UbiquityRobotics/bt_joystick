# Bluetooth BLE Joystick node for the Magicsee R1 joystick in gamepad mode =

This code implemente a ROS node to interface a Magicsee R1 ring-held Bluetooth LE joystick/gamepad for /cmd_vel robot control

The hand controller notifications are received by this node for the joystick and buttons.
This node then publishes to ROS topic with 'twist' messages normally we use topic /cmd_vel
The default /cmd_vel topic and speeds may be changed using parameters

Configuration is done in:  catkin_ws/src/demos/bt_joystick/bt_joystick.yaml
The BT MAC address MUST match the hand controller to be used.
Use App like nRFConnect and get address for 'Magicsee R1' (usually starts with FF:FF:80:06)

WARNING:  You MUST start the Bt Joystick first AND simul-press 'M' + 'B' before launching node!

## Dependencies

There are many system level configuration issues that must be present for this node to operate.
In summary the python gattlib library is used and there is other setup.
This README does not yet explain the setup as this node is to be included on the Ubiquity Robotics platforms that use a Raspberry Pi 3 with the kernel and system configuration required to run this node.

At a minimum, the following commands are necessary:

```
sudo apt install libbluetooth-dev
pip install gattlib
```

## Configuration

This node must have as a minimum the ROS parameter setup in bt_joystick.yaml for the controller in use.
You may run this command on a configured system to find the Magicsee R1 MAC address then use it.

    hcitool lescan               (After you find the Magicsee you may use Control-C)

Here is an explanation of the parameters in bt_joystick.yaml

    ROS Param            Default  Description
    bt_mac_address  FF:FF:80:06:6C:59     Bluetooth LE MAC Address (EACH unit is different MAC Addr)
    speed_stopped           0.0   Robot X speed when stopped (can be used to null out robot offset)
    speed_fwd_normal        0.2   Robot forward X speed in M/Sec for joystick forward straight
    speed_fwd_turning       0.2   Robot forward X speed in M/Sec for joystick in turn mode (not rotate mode)
    speed_rev_normal       -0.15  Robot reverse X speed in M/Sec for joystick reverse straight back
    speed_turbo_multiplier  1.75  Speed multiplier for X speed if in a mode where X is non-zero
    angular_straight_rate   0.0   Robot Z angular rate when stopped (can be used to null out robot offset)
    angular_turning_rate    0.3   Robot Z angular rate in Rad/Sec when turning (not for rotate)
    angular_rotate_rate     0.4   Robot Z angular rate in Rad/Sec when rotating

## System Limitations

Beside having ROS environment and bt_joystick node configured,
there are known design limitation or key operational thing to be known as follows

    - Must start bluetooth.service with -compat and some config for a user in group bluetooth to use
    - User MUST hit M and B after EACH power-up of the controller to get to gamepad mode!
    - Must have the Bluetooth MAC address set in bt_joystick.yaml as  bt_mac_address rosparam
    - Must NOT have any device like Android paired to the Magicsee R1 or this node is shut out

