#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import evdev
import time
from pprint import pprint

# Replace '/dev/input/eventX' with your actual joystick device file
DEVICE_PATH = "/dev/input/by-path/pci-0000:00:14.0-usb-0:2:1.0-event-joystick"  # Example path, change to match your joystick device

def joy_publisher():
    # Initialize the ROS node
    rospy.init_node('joy_publisher', anonymous=True)
    pub = rospy.Publisher('joy', Joy, queue_size=1)
    
    # Open the joystick device using evdev
    try:
        dev = evdev.InputDevice(DEVICE_PATH)
    except FileNotFoundError:
        rospy.logerr(f"Device {DEVICE_PATH} not found.")
        return
    
    rospy.loginfo(f"Reading from {DEVICE_PATH}")

    # Initialize Joy message to publish
    joy_msg = Joy()
    
    # Map for the number of axes and buttons
    abilites = dev.capabilities(absinfo=True, verbose=True) # Number of axes
    axes_to_index = {} # name of axis:index
    buttons_to_index = {}
    pprint(abilites)
    axes = abilites[('EV_ABS', 3)]
    buttons = abilites[('EV_KEY', 1)]

    for idx, val in enumerate(axes):
        axes_to_index[val[0][0]] = idx  
    for idx, val in enumerate(buttons):
        print(idx, val)
        val_type = type(val[0])
        if val_type == str: 
            buttons_to_index[val[0]] = idx
        elif val_type == list:
            buttons_to_index[val[0][0]] = idx
        else:
            assert False, "should be str or list"
    print(axes_to_index)
    print(buttons_to_index)
    print(type(axes))
    num_axes = len(axes)
    num_buttons = len(buttons)
    print(f"This many axis {num_axes}, buttons {num_buttons}")
    #exit()

    joy_msg.axes = [0.0] * num_axes
    joy_msg.buttons = [0] * num_buttons
    
    while not rospy.is_shutdown():
        # Read events from the joystick
        event = dev.read_one()
        if event:
            # Process the event if it's an ABS (axis) or KEY (button) event
            #print(event.type)
            if event.type == evdev.ecodes.EV_ABS:
                axis = evdev.ecodes.ABS[event.code]
                if abs(event.value) < 2500:
                    value = 0
                else:
                    value = event.value / 32767.0  # Normalize to -1.0 to 1.0
                #print(f"{axis} val {value}, {event}")
                #print(f"postion in array {axes[]}")
                joy_msg.axes[axes_to_index[axis]] = value
            elif event.type == evdev.ecodes.EV_KEY:

                    if event.code in evdev.ecodes.KEY: 
                        button = evdev.ecodes.KEY[event.code] 
                    elif event.code in evdev.ecodes.BTN: 
                        button = evdev.ecodes.BTN[event.code] 
                    value = 1 if event.value == 1 else 0
                    #print(f"button {button} value {value} {event}")
                    if type(button) == list:
                        button = button[0]
                    assert type(button) == str, "Expected str" 
                    joy_msg.buttons[buttons_to_index[button]] = value


            # Publish the Joy message
            pub.publish(joy_msg)
            #rospy.sleep(0.01)  # Sleep to control the publish rate

if __name__ == '__main__':
    try:
        print(dict(sorted(evdev.ecodes.KEY.items())))
        print("\n\n\n\n")
        print(dict(sorted(evdev.ecodes.BTN.items())))

        joy_publisher()
    except rospy.ROSInterruptException:
        pass
