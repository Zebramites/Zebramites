#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import evdev
import time

# Replace '/dev/input/eventX' with your actual joystick device file
DEVICE_PATH = "/dev/input/event17"  # Example path, change to match your joystick device

def joy_publisher():
    # Initialize the ROS node
    rospy.init_node('joy_publisher', anonymous=True)
    pub = rospy.Publisher('joy', Joy, queue_size=10)
    
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
    num_axes = len(dev.capabilities(evdev.ecodes.EV_ABS))  # Number of axes
    num_buttons = len(dev.capabilities(evdev.ecodes.EV_KEY))  # Number of buttons
    
    joy_msg.axes = [0.0] * num_axes
    joy_msg.buttons = [0] * num_buttons
    
    while not rospy.is_shutdown():
        # Read events from the joystick
        for event in dev.read_loop():
            # Process the event if it's an ABS (axis) or KEY (button) event
            
            print(event.type)
            if event.type == evdev.ecodes.EV_ABS:
                axis = evdev.ecodes.ABS[event.code]
                value = event.value / 32767.0  # Normalize to -1.0 to 1.0
                print(f"{axis} val {value}, {event}")
                # joy_msg.axes[axis] = value
            elif event.type == evdev.ecodes.EV_KEY:
                if event.code in evdev.ecodes.KEY: 
                    button = evdev.ecodes.KEY[event.code] 
                elif event.code in evdev.ecodes.BTN: 
                    button = evdev.ecodes.BTN[event.code] 
                value = 1 if event.value == 1 else 0
                print(f"button {button} value {value} {event}")

                # joy_msg.buttons[button] = value

        # Publish the Joy message
        pub.publish(joy_msg)
        rospy.sleep(0.01)  # Sleep to control the publish rate

if __name__ == '__main__':
    try:
        print(dict(sorted(evdev.ecodes.KEY.items())))
        print("\n\n\n\n")
        print(dict(sorted(evdev.ecodes.BTN.items())))

        joy_publisher()
    except rospy.ROSInterruptException:
        pass
