#!/usr/bin/env python3

import rospy
from plutodrone.msg import PlutoMsg
import os
os.environ["SDL_AUDIODRIVER"] = "dummy"
import pygame  # Import Pygame

def arm_drone(command):
    command.rcYaw = 1500
    command.rcPitch = 1500
    command.rcRoll = 1500
    command.rcThrottle = 1000 #start at smallest throttle
    command.rcAUX1 = 1000
    command.rcAUX2 = 1000 # start without dev-mode
    command.rcAUX3 = 1000 # start without alt-lock
    command.rcAUX4 = 1500

def disarm_drone(command):
    command.rcAUX4 = 1000
        

def main():
    # Initialize the ROS node
    rospy.init_node('drone_command_publisher', anonymous=True)

    # Create a publisher for the /drone_command topic
    pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

    # Create a rate object to control the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize command values
    command = PlutoMsg()
    disarm_drone(command)
    #variables
    delta = 50 #change in single keypress
    default = 1500 #reset value
    max_val = 2000 #upper limit 
    min_val = 1000 #lower limit
    armed = False #status of drone
    alt_lock = False #altitude lock of drone

    print("Use the keyboard to control the drone. Press 'q' to quit.")
    print("w/s: Increase/Decrease Pitch | a/d: Increase/Decrease Roll")
    print("i/k: Increase/Decrease Throttle | j/l: Increase/Decrease Yaw")
    print("e: Altitude Lock/Unlock")
    print("SpaceBar: To Arm/Disarm Drone")
    print("To Start Arm the Drone")

    pygame.init()
    screen = pygame.display.set_mode((640, 480))  # Create a dummy window
    pygame.display.set_caption("Drone Controller")  # Set window title

    while not rospy.is_shutdown():
        for event in pygame.event.get():  # Process Pygame events
            if event.type == pygame.QUIT:  # Check for window close
                rospy.signal_shutdown("Window closed.")
        
        keys = pygame.key.get_pressed()  # Get the current state of all keys

        # Check if keys are pressed
        if keys[pygame.K_SPACE]:
            if armed:
                disarm_drone(command)
                armed = False
                print("Drone is disarmed")
            else:
                arm_drone(command)
                armed = True
        if armed:
            if keys[pygame.K_w]:
                command.rcPitch += delta
            if keys[pygame.K_s]:
                command.rcPitch -= delta
            if not keys[pygame.K_w] and not keys[pygame.K_s]:
                command.rcPitch = default

            if keys[pygame.K_a]:
                command.rcRoll -= delta
            if keys[pygame.K_d]:
                command.rcRoll += delta
            if not keys[pygame.K_a] and not keys[pygame.K_d]:
                command.rcRoll = default
            
            if keys[pygame.K_e]:
                alt_lock = not alt_lock
                # 1300 < AUX3 < 1700 : Alt Hold mode on.
                # Outside the above range : Throttle range.
                if alt_lock:
                    command.rcAUX3 = 1500 
                else:
                    command.rcAUX3 = 1000
            if not alt_lock: #change throttle only when alt_lock disabled
                if keys[pygame.K_i]:
                    command.rcThrottle += delta
                if keys[pygame.K_k]:
                    command.rcThrottle -= delta
                # if not keys[pygame.K_i] and not keys[pygame.K_k]:
                #     command.rcPitch = default
            else:
                print("Alt Lock Mode", end = " ")

            if keys[pygame.K_j]:
                command.rcYaw -= delta
            if keys[pygame.K_l]:
                command.rcYaw += delta
            # if not keys[pygame.K_j] and not keys[pygame.K_l]:
            #     command.rcPitch = default

            command.rcYaw = max(min_val,command.rcYaw)
            command.rcYaw = min(max_val,command.rcYaw)
            command.rcPitch = max(min_val,command.rcPitch)
            command.rcPitch = min(max_val,command.rcPitch)
            command.rcRoll = max(min_val,command.rcRoll)
            command.rcRoll = min(max_val,command.rcRoll)
            command.rcThrottle = max(min_val,command.rcThrottle)
            command.rcThrottle = min(max_val,command.rcThrottle)

        if keys[pygame.K_q]:
            disarm_drone(command)
            print("Disarming Drone")
            pub.publish(command)
            print("Quitting...")
            break
        
        # Print the current command values
        if armed:
            print(f"Current Command: rcYaw: {command.rcYaw}, rcPitch: {command.rcPitch}, rcRoll: {command.rcRoll}, rcThrottle: {command.rcThrottle}")
        # Publish the updated command
        pub.publish(command)

        # Sleep for the remainder of the loop cycle
        rate.sleep()

    # Cleanup
    pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3

# import rospy
# from plutodrone.msg import PlutoMsg
# import keyboard  # Import the keyboard library

# def main():
#     # Initialize the ROS node
#     rospy.init_node('drone_command_publisher', anonymous=True)

#     # Create a publisher for the /drone_command topic
#     pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

#     # Create a rate object to control the loop rate
#     rate = rospy.Rate(10)  # 10 Hz

#     # Initialize command values
#     command = PlutoMsg()
#     command.rcYaw = 1500
#     command.rcPitch = 1500
#     command.rcRoll = 1500
#     command.rcThrottle = 1500
#     command.rcAUX1 = 1500
#     command.rcAUX2 = 1500
#     command.rcAUX3 = 1500
#     command.rcAUX4 = 1500

#     print("Use the keyboard to control the drone. Press 'q' to quit.")
#     print("w/s: Increase/Decrease Pitch | a/d: Increase/Decrease Roll")
#     print("i/k: Increase/Decrease Throttle | j/l: Increase/Decrease Yaw")

#     while not rospy.is_shutdown():
#         # Check if keys are pressed
#         if keyboard.is_pressed('w'):
#             command.rcPitch += 50
#         if keyboard.is_pressed('s'):
#             command.rcPitch -= 50
#         if not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
#             command.rcPitch = 1500
#         if keyboard.is_pressed('a'):
#             command.rcRoll -= 50
#         if keyboard.is_pressed('d'):
#             command.rcRoll += 50
#         if not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
#             command.rcPitch = 1500
#         if keyboard.is_pressed('i'):
#             command.rcThrottle += 50
#         if keyboard.is_pressed('k'):
#             command.rcThrottle -= 50
#         # if not keyboard.is_pressed('i') and not keyboard.is_pressed('k'):
#         #     command.rcPitch = 1500
#         if keyboard.is_pressed('j'):
#             command.rcYaw -= 50
#         if keyboard.is_pressed('l'):
#             command.rcYaw += 50
#         # if not keyboard.is_pressed('j') and not keyboard.is_pressed('l'):
#         #     command.rcPitch = 1500
#         if keyboard.is_pressed('q'):
#             print("Quitting...")
#             break
        
#         command.rcYaw = max(1000,command.rcYaw)
#         command.rcYaw = min(2000,command.rcYaw)
#         command.rcPitch = max(1000,command.rcPitch)
#         command.rcPitch = min(2000,command.rcPitch)
#         command.rcRoll = max(1000,command.rcRoll)
#         command.rcRoll = min(2000,command.rcRoll)
#         command.rcThrottle = max(1000,command.rcThrottle)
#         command.rcThrottle = min(2000,command.rcThrottle)
#         # Print the current command values
#         print(f"Current Command: rcYaw: {command.rcYaw}, rcPitch: {command.rcPitch}, rcRoll: {command.rcRoll}, rcThrottle: {command.rcThrottle}")

#         # Publish the updated command
#         pub.publish(command)

#         # Sleep for the remainder of the loop cycle
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass



# #!/usr/bin/env python3

# import rospy
# from plutodrone.msg import PlutoMsg
# import sys
# import termios
# import tty

# # Function to get keyboard input
# def getKey():
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(sys.stdin.fileno())
#         key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key

# def main():
#     # Initialize the ROS node
#     rospy.init_node('drone_command_publisher', anonymous=True)

#     # Create a publisher for the /drone_command topic
#     pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

#     # Create a rate object to control the loop rate
#     rate = rospy.Rate(10)  # 10 Hz

#     # Initialize command values
#     command = PlutoMsg()
#     command.rcYaw = 0
#     command.rcPitch = 0
#     command.rcRoll = 0
#     command.rcThrottle = 0
#     command.rcAUX1 = 0
#     command.rcAUX2 = 0
#     command.rcAUX3 = 0
#     command.rcAUX4 = 0

#     print("Use the keyboard to control the drone. Press 'q' to quit.")
#     print("w/s: Increase/Decrease Pitch | a/d: Increase/Decrease Roll")
#     print("i/k: Increase/Decrease Throttle | j/l: Increase/Decrease Yaw")

#     while not rospy.is_shutdown():
#         key = getKey()
#         command.rcYaw = 0
#         command.rcPitch = 0
#         command.rcRoll = 0
#         command.rcThrottle = 0
#         command.rcAUX1 = 0
#         command.rcAUX2 = 0
#         command.rcAUX3 = 0
#         command.rcAUX4 = 0
#         if key == 'w':
#             command.rcPitch += 10
#             print("Increase Pitch:", command.rcPitch)
#         elif key == 's':
#             command.rcPitch -= 10
#             print("Decrease Pitch:", command.rcPitch)
#         elif key == 'a':
#             command.rcRoll -= 10
#             print("Decrease Roll:", command.rcRoll)
#         elif key == 'd':
#             command.rcRoll += 10
#             print("Increase Roll:", command.rcRoll)
#         elif key == 'i':
#             command.rcThrottle += 10
#             print("Increase Throttle:", command.rcThrottle)
#         elif key == 'k':
#             command.rcThrottle -= 10
#             print("Decrease Throttle:", command.rcThrottle)
#         elif key == 'j':
#             command.rcYaw -= 10
#             print("Decrease Yaw:", command.rcYaw)
#         elif key == 'l':
#             command.rcYaw += 10
#             print("Increase Yaw:", command.rcYaw)
#         elif key == 'q':
#             print("Quitting...")
#             break
#         # if key == 'w':
#         #     command.rcPitch = 1
#         #     print("Increase Pitch:", command.rcPitch)
#         # if key == 's':
#         #     command.rcPitch =2
#         #     print("Decrease Pitch:", command.rcPitch)
#         # if key == 'a':
#         #     command.rcPitch =3
#         #     print("Decrease Roll:", command.rcPitch)
#         # if key == 'd':
#         #     command.rcPitch =4
#         #     print("Increase Roll:", command.rcPitch)
#         # if key == 'i':
#         #     command.rcPitch =5
#         #     print("Increase Throttle:", command.rcPitch)
#         # if key == 'k':
#         #     command.rcPitch =6
#         #     print("Decrease Throttle:", command.rcPitch)
#         # if key == 'j':
#         #     command.rcPitch =7
#         #     print("Decrease Yaw:", command.rcPitch)
#         # if key == 'l':
#         #     command.rcPitch =8
#         #     print("Increase Yaw:", command.rcPitch)
#         # if key == 'q':
#         #     command.rcPitch =9
#         #     print("Increase Yaw:", command.rcPitch)
#         #     print("Quitting...")
#         #     break
#         # print(key)
#         # Publish the updated command
#         pub.publish(command)

#         # Sleep for the remainder of the loop cycle
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
