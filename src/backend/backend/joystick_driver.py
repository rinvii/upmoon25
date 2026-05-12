# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import pygame

from rclpy.node import Node

from pygame.locals import QUIT

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# Controller Axes (return -1.0 to 1.0)
LJOY_X = 0
LJOY_Y = 1
LBUMPER = 2
RJOY_X = 3
RJOY_Y = 4
RBUMPER = 5

# Controller Buttons
A_BT = 0
B_BT = 1
X_BT = 2
Y_BT = 3
LTRIGGER = 4
RTRIGGER = 5
BACK_BT = 6
START_BT = 7
POWER_BT = 8
LSTICK_BT = 9
RSTICK_BT = 10

'''
    This node allows the user to control the robot using a joystick.
    TODO: which buttons do what?

    Publishes:
    /cmd/velocity - Twist, linear.x for forward/backward speed, angular.z for rotation speed
    /cmd/conveyor - Int16, 1 to turn on the conveyor, 0 to turn off
    /cmd/bucket_vel - Int16, speed of the bucket chain
    /cmd/bucket_pos - Int16, position of the bucket chain
    /cmd/pan - Int16, +1 for pan left, -1 for pan right, 0 for stop
    /cmd/camera_height - Int16, 0 - 100 height percentage for the camera
'''
class JoystickDriver(Node):

    def __init__(self):
        super().__init__('joystick_driver')

        # How frequently pygame should be polled
        self.clk = 0.1

        #topics
        self.velocity_pub = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.conveyor_pub = self.create_publisher(Int16, 'cmd/conveyor', 10)
        self.bucket_vel_pub = self.create_publisher(Int16, 'cmd/bucket_vel', 10)
        self.bucket_pos_pub = self.create_publisher(Int16, 'cmd/bucket_pos', 10)
        self.camera_pan_pub = self.create_publisher(Int16, 'cmd/pan', 1)
        self.camera_height_pub = self.create_publisher(Int16, 'cmd/camera_height', 10)

        self.bucket_pos = 0.0
        self.bucket_auto_on = False
        self.bucket_auto_debounce = 0

        # Timer for polling events from pygame
        self.timer = self.create_timer(self.clk, self.pollEvents)

        pygame.init()
        pygame.joystick.init()

        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('testbot driver')

        self.controller = pygame.joystick.Joystick(0)

        num_buttons = self.controller.get_numbuttons()
        num_axes = self.controller.get_numaxes()
        print("Buttons: " + str(num_buttons) + " Axes: " + str(num_axes))

        self.FPS = 25

    # Publishes joystick inputs
    def pollEvents(self):
        # For Debugging: print all of the axes/button states
        # print("Axis 0: " + str(self.controller.get_axis(0)))
        # print("Axis 1: " + str(self.controller.get_axis(1)))
        # print("Axis 2: " + str(self.controller.get_axis(2)))
        # print("Axis 3: " + str(self.controller.get_axis(3)))
        # print("Axis 4: " + str(self.controller.get_axis(4)))
        # print("Axis 5: " + str(self.controller.get_axis(5)))
        # print("Button 0: " + str(self.controller.get_button(0)))
        # print("Button 1: " + str(self.controller.get_button(1)))
        # print("Button 2: " + str(self.controller.get_button(2)))
        # print("Button 3: " + str(self.controller.get_button(3)))
        # print("Button 4: " + str(self.controller.get_button(4)))
        # print("Button 5: " + str(self.controller.get_button(5)))
        # print("Button 6: " + str(self.controller.get_button(6)))
        # print("Button 7: " + str(self.controller.get_button(7)))
        # print("Button 8: " + str(self.controller.get_button(8)))
        # print("Button 9: " + str(self.controller.get_button(9)))
        # print("Button 10: " + str(self.controller.get_button(10)))

        for event in pygame.event.get():
            if event.type == QUIT:
                self.destroy_node()
                rclpy.shutdown()
                pygame.quit()


        # Publish Velocity Command
        throttle = -100 * self.controller.get_axis(LJOY_Y)
        if (throttle <= 5 and throttle >= -5):
            throttle = 0.0
        
        rotation = 0.0
        if self.controller.get_button(LTRIGGER):
            rotation -= 50.0
        if self.controller.get_button(RTRIGGER):
            rotation += 50.0

        msg = Twist()
        msg.linear.x = throttle
        msg.angular.z = rotation
        self.velocity_pub.publish(msg)


        # Publish Conveyor Command
        msg = Int16()
        msg.data = self.controller.get_button(Y_BT)
        self.conveyor_pub.publish(msg)

        # Publish Bucket Chain Position
        msg = Int16()
        if (self.controller.get_button(X_BT) and self.bucket_pos < 100.0):
            self.bucket_pos += 0.5
            msg.data = int(self.bucket_pos)
            self.bucket_pos_pub.publish(msg)
        elif (self.controller.get_button(B_BT) and self.bucket_pos > 0.0):
            self.bucket_pos -= 0.5
            msg.data = int(self.bucket_pos)
            self.bucket_pos_pub.publish(msg)

        # Publish Bucket Chain Speed
        self.bucket_auto_debounce += 1
        if (self.controller.get_button(A_BT) and self.bucket_auto_debounce > 6):
            self.bucket_auto_on = ~self.bucket_auto_on
            self.bucket_auto_debounce = 0

        if (self.bucket_auto_on):
            bucket_speed = 100
        else:
            bucket_speed = int(-100.0 * self.controller.get_axis(RJOY_Y))
            if (abs(bucket_speed) <= 10):
                bucket_speed = 0
            elif (abs(bucket_speed) <=20):
                bucket_speed = 20 if bucket_speed > 0 else -20
            elif (abs(bucket_speed) <= 40):
                bucket_speed = 40 if bucket_speed > 0 else -40
            elif (abs(bucket_speed) <=60):
                bucket_speed = 60 if bucket_speed > 0 else -60
            elif (abs(bucket_speed) <= 80):
                bucket_speed = 80 if bucket_speed > 0 else -80
            elif (abs(bucket_speed) <= 100):
                bucket_speed = 100 if bucket_speed > 0 else -100

        msg = Int16()
        msg.data = bucket_speed
        self.bucket_vel_pub.publish(msg)
            
        msg = Int16()
        if (self.controller.get_button(BACK_BT)):
            msg.data = 1
        elif (self.controller.get_button(START_BT)):
            msg.data = -1
        else:
            msg.data = 0

        self.camera_pan_pub.publish(msg)  

        # Auto set camera height to 100
        msg = Int16()
        msg.data = 100
        self.camera_height_pub.publish(msg)

        pygame.display.update()
        pygame.time.Clock().tick(self.FPS)


def main(args=None):
    rclpy.init(args=args)

    joystick_driver = JoystickDriver()

    rclpy.spin(joystick_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
