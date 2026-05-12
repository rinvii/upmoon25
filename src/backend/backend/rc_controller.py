import rclpy
import pygame

from rclpy.node import Node

from pygame.locals import KEYDOWN, KEYUP, QUIT

from geometry_msgs.msg import Twist
from std_msgs.msg import String

'''
    Keyboard controls for simulator robot.

    wasd - motion controls
    zx   - camera servo controls
'''
class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # How frequently pygame should be polled
        self.clk = 0.01

        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub    = self.create_publisher(String, '/cmd_servo', 10)

        # Timer for polling events from pygame
        self.timer = self.create_timer(self.clk, self.pollEvents)

        pygame.init()
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 30)	

        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('bbot driver')

        self.throttle = 100.0

        self.FPS = 60

        self.updateThrottleText()

        self.get_logger().info("Succesful initialization")

    def updateThrottleText(self):
        text_surface = self.font.render(f'Throttle: {self.throttle}', False, (255, 255, 255))
        self.screen.fill((0,0,0))
        self.screen.blit(text_surface, (0,0)) 


    # Helper method for pollEvents
    def setKeys(self, key, val):
        velocity_msg = Twist()
        servo_msg    = String()

        if (val == 0):
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            servo_msg.data = "0"
        else:
            if (key == 119): # w
                velocity_msg.linear.x = self.throttle / 50.0 # set the direction to positive
            if (key == 115): # s
                velocity_msg.linear.x = -self.throttle / 50.0 # set the throttle to negative
            if (key == 97): # a
                velocity_msg.angular.z = -2.0 # negative value indicates a left turn
                #velocity_msg.linear.x = self.throttle # set the direction to positive
            if (key == 100): # d
                velocity_msg.angular.z = 2.0 # positive value indicates a right turn
                #velocity_msg.linear.x = self.throttle # set the direction to positive
            if (key == 101):
                self.throttle += 10.0
                if (self.throttle > 100.0):
                    self.throttle = 100.0
                self.updateThrottleText()
            if (key == 113):
                self.throttle -= 10.0
                if (self.throttle < 0):
                    self.throttle = 0
                self.updateThrottleText()
            if (key == 122): #z
                servo_msg.data = "1"
            if (key == 120): #x
                servo_msg.data = "2"

        # Velocity data is published as a twist object. The linear x component represents throttle (+/- = forward/backward)
        # The angular z component represents turning (+/- = right/left, 0 = no turn). Turning speed is constant regardless of z magnitude
        self.velocity_pub.publish(velocity_msg)
        self.servo_pub.publish(servo_msg)

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

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()