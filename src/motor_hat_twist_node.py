#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# sudo pip3 install adafruit-circuitpython-motorkit
from adafruit_motorkit import MotorKit

from matrix import Matrix

kit = MotorKit()

jacobian = Matrix(4, 3)
direction = Matrix(3, 1)

def callback(vel_msg):
    """
        note: throttle= [-1; 1]
    """
    global kit, jacobian, direction

    direction[0, 0] = vel_msg.linear.x
    direction[1, 0] = vel_msg.linear.y
    direction[2, 0] = vel_msg.angular.z

    print("direction\n", direction)
    print("jacobian\n", jacobian)

    motors_cmd = jacobian * direction

    kit.motor1.throttle = motors_cmd[0, 0]
    kit.motor2.throttle = motors_cmd[1, 0]
    kit.motor3.throttle = motors_cmd[2, 0]
    kit.motor4.throttle = motors_cmd[3, 0]

    print(direction)
    print(motors_cmd)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_hat_twist_node', anonymous=True)

    topic_name = rospy.get_param('cmd_topic_name', '/pogo/cmd_vel')

    rospy.Subscriber(topic_name, Twist, callback)

    rospy.loginfo("Starting listening topic "+topic_name)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    for i in range(jacobian.nb_lines):
        for j in range(jacobian.nb_columns):
            jacobian[i, j] = 1

    jacobian[0, 0] = -1
    jacobian[1, 0] = -1
    jacobian[0, 2] = -1
    jacobian[2, 2] = -1
    print(jacobian)



    listener()

