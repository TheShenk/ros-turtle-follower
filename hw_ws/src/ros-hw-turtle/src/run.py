#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def sign(x):
    return -1 if x < 0 else 1

class TurtleFollower:

    def __init__(self):
        rospy.init_node('follower', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/donatello/cmd_vel', Twist, queue_size=10)
        self.donatello_pos_subscriber = rospy.Subscriber('/donatello/pose', Pose, self.update_donatello_pos)
        self.leonardo_pos_subscriber = rospy.Subscriber('/leonardo/pose', Pose, self.update_leonardo_pos)

        self.donatello_position = Pose()
        self.leonardo_position = Pose()
        self.rate = rospy.Rate(100)

    def run(self):

        while not rospy.is_shutdown():
            direction_vector = [self.leonardo_position.x - self.donatello_position.x,
                                self.leonardo_position.y - self.donatello_position.y]
            distance = math.sqrt(math.pow(direction_vector[0], 2) + math.pow(direction_vector[1], 2))

            angle = math.atan2(direction_vector[1], direction_vector[0]) - self.donatello_position.theta

            # При малых значениях компонент вектора перемещения угол слишком часто перескакивает с
            # положительных к отрицательным и наоборот, из-за чего постоянно крутится в разные стороны.
            # Поэтому требуем, что бы в таких случаях всегда крутился в одну сторону
            if angle < 0 and (abs(direction_vector[0]) < 0.05 or abs(direction_vector[1]) < 0.05):
                angle = 2*math.pi + angle

            linear_slow_coeff = min(distance, 1.0)

            message = Twist()
            if abs(angle) < 0.1:
                message.linear.x =  2 * linear_slow_coeff
            else:
                message.angular.z =  4 * sign(angle)

            self.velocity_publisher.publish(message)
            self.rate.sleep()

    def update_donatello_pos(self, pos):
        self.donatello_position = pos

    def update_leonardo_pos(self, pos):
        self.leonardo_position = pos

if __name__ == '__main__':
    follower = TurtleFollower()
    follower.run()
