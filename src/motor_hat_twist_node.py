#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# sudo pip3 install adafruit-circuitpython-motorkit
from adafruit_motorkit import MotorKit

kit = MotorKit()

def callback(vel_msg):
    """
        note: throttle= [-1; 1]
    """
    global kit

    if vel_msg.linear.x != 0:
        # avancer
        kit.motor1.throttle = vel_msg.linear.x
        kit.motor2.throttle = vel_msg.linear.x
        kit.motor3.throttle = vel_msg.linear.x
        kit.motor4.throttle = vel_msg.linear.x
    elif vel_msg.linear.y != 0:
        kit.motor1.throttle = vel_msg.linear.y
        kit.motor2.throttle = vel_msg.linear.y
        kit.motor3.throttle = vel_msg.linear.y
        kit.motor4.throttle = vel_msg.linear.y
    elif vel_msg.angular.z != 0:
        kit.motor1.throttle = vel_msg.angular.z
        kit.motor2.throttle = vel_msg.angular.z
        kit.motor3.throttle = vel_msg.angular.z
        kit.motor4.throttle = vel_msg.angular.z

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_hat_twist_node', anonymous=True)

    topic_name = rospy.get_param('cmd_topic_name', '/pogo/cmd_vel')

    rospy.Subscriber(topic_name, Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

