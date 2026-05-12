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

from pygame.locals import KEYDOWN, KEYUP, QUIT

from std_msgs.msg import Int16, Int8


class MinimalDriver(Node):

    def __init__(self):
        super().__init__('minimal_driver')

        # How frequently pygame should be polled
        self.clk = 0.01

        self.pan_pub = self.create_publisher(Int16, 'cmd/pan', 3)
        self.cam_height_pub = self.create_publisher(Int16, 'cmd/camera_height', 10)
        self.pc_pub = self.create_publisher(Int8, 'cmd/pointcloud', 1)


        # Timer for polling events from pygame
        self.timer = self.create_timer(self.clk, self.pollEvents)

        pygame.init()
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 30)	

        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('bbot driver')

        self.throttle = 100.0

        self.cam_height = 0

        self.FPS = 60

        self.updateThrottleText()

        self.get_logger().info("Succesful initialization")

    def updateThrottleText(self):
        text_surface = self.font.render(f'Throttle: {self.throttle}', False, (255, 255, 255))
        self.screen.fill((0,0,0))
        self.screen.blit(text_surface, (0,0)) 


    # Helper method for pollEvents
    def setKeys(self, key, val):
        

        if (val == 0):
            pass
        else:
            if (key == 117): # u
                self.cam_height += 1
                if (self.cam_height > 100):
                    self.cam_height = 100
            if (key == 106): # j
                self.cam_height -= 1
                if (self.cam_height < 0):
                    self.cam_height = 0
            if (key == ord('p')):
                pc_msg = Int8()
                pc_msg.data = 1
                self.pc_pub.publish(pc_msg)

        height_msg = Int16()
        height_msg.data = self.cam_height
        self.cam_height_pub.publish(height_msg)

    # Publishes keyboard inputs
    def pollEvents(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()

            elif event.type == KEYDOWN:
                key = event.__dict__['key']
                self.setKeys(key, 1)

            elif (event.type == KEYUP):
                key = event.__dict__['key']
                self.setKeys(key, 0)

            pygame.display.update()
            pygame.time.Clock().tick(60)


def main(args=None):
    rclpy.init(args=args)

    minimal_driver = MinimalDriver()

    rclpy.spin(minimal_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
