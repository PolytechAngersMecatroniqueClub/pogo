# general imports
import os, sys, inspect, thread, time
import psutil

# ROS imports
import rospy
from geometry_msgs.msg import Twist

# Path to the LeapMotion SDK/Libraries
# Needs to be changed if not the same PC...
sys.path.append("/home/remy/Downloads/Leap_Motion_SDK_Linux_2.3.1/LeapDeveloperKit_2.3.1+31549_linux/LeapSDK/lib/x64")
sys.path.append("/home/remy/Downloads/Leap_Motion_SDK_Linux_2.3.1/LeapDeveloperKit_2.3.1+31549_linux/LeapSDK/lib")

# import the leap motion library
import Leap


class Parameters:
    def __init__(self):
        self.min_speed = 0
        self.max_speed = 0
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.min_theta = 0
        self.max_theta = 0

def normalize(min_val, max_val, val):
    """ function to normalize a value between [-1, 1]  
        according to the max and min of the value
            :param min_val float: the minimal value for val
            :param max_val float: the maximal value for val
            :param val float: the value to normalize
            :return float: the normalized value
    """
    val = min(val, max_val)
    val = max(val, min_val)

    return val / max(abs(max_val), abs(min_val))

def main_loop(pub, rate, params):
    """ Main loop function of the node
        It connects to the LeapMotion, read the value, normalize them
        and send the twist
            :param pub rospy.topics.Publisher: The twist velocity command publisher
            :param rate rospy.timer.Rate: The rate to publish the command
            :param params Parameters: The parameters for the normalizations (min and max values)
    """

    # connect to the leap motion
    controller = Leap.Controller()

    # to get an information about the leapmotion connection
    flag_connected = False

    # main loop of the node, runs until the node is shutting down
    while not rospy.is_shutdown():
        # test the LeapMotion connection to display warning if it is not connected
        if not controller.is_connected:
            rospy.logwarn("Not connected to the LeapMotion")
            flag_connected = False
        elif not flag_connected:
            rospy.loginfo("Connected to the LeapMotion")
            flag_connected = True
            
        # create the message that will be send
        cmd_vel = Twist()
        # get the value of the LeapMotion
        frame = controller.frame()

        # test if one and only one hand is over the sensor
        if len(frame.hands) == 1:
            # get the hand parameters (position and orientation)
            hand = frame.hands[0]
            # get the speed according to the hight of the hand above the sensor
            speed = normalize(params.min_speed, params.max_speed, hand.palm_position[1])
            # get the twist parameters (orientation translation) according to the hand parameters
            cmd_vel.linear.x = normalize(params.min_x, params.max_x, -hand.palm_position[2])
            cmd_vel.linear.y =  normalize(params.min_y, params.max_y, hand.palm_position[0])
            cmd_vel.angular.z = normalize(params.min_theta, params.max_theta, hand.direction[0])
        else:
            # otherwise the speed is set to 0 to stop the robot
            speed = 0

        # limit the value of the Twist according to the desired speed
        cmd_vel.linear.x *= speed
        cmd_vel.linear.y *= speed
        cmd_vel.angular.z *= speed

        # publish the Twist message
        pub.publish(cmd_vel)

        # sleep before looping for the 10Hz rate
        rate.sleep()



if __name__ == "__main__":
    # if this python file is executed as main file

    # create the ROS node
    rospy.init_node('leap_to_twist_node', anonymous=False)

    # get the parameters from the launch file
    # the second value is the default value
    params.min_speed = rospy.get_param('min_speed', 0)
    params.max_speed = rospy.get_param('max_speed', 300)
    params.min_x =     rospy.get_param('min_x',    -100)
    params.max_x =     rospy.get_param('max_x',     100)
    params.min_y =     rospy.get_param('min_y',    -100)
    params.max_y =     rospy.get_param('max_y',     100)
    params.min_theta = rospy.get_param('min_theta',-0.9)
    params.max_theta = rospy.get_param('max_theta', 0.9)
    topic_name =       rospy.get_param('cmd_topic_name', '/pogo/cmd_vel')





    # create the publisher for the velocity twist
    pub = rospy.Publisher(topic_name, Twist, queue_size=10)
    # to sleep in the loop, the Twist will be publised at a 10Hz rate
    rate = rospy.Rate(10)  # 10 Hz

    params = Parameters()

    # The Leap motion library can only be used with python 2.7...
    if sys.version_info[0] > 2 or sys.version_info[1] < 6:
        rospy.logerr("You need Python 2.7 to run this script")
        quit()

    # The programm leapd should be started to use the LeapMotion
    if not ("leapd" in (p.name() for p in psutil.process_iter())):
        rospy.logerr("You need to start <sudo leapd> first!")
        quit()

    # Start the main loop of the node
    try:
        main_loop(pub=pub, rate=rate, params=params)
    except rospy.ROSInterruptException as _:
        pass
