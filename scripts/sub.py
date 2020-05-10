#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def print_message(message):
    rospy.loginfo(message)

def main():
    try:
        rospy.init_node('palchukovsky_subscriber', anonymous=True)
        rospy.Subscriber("/demo/text", String, print_message)
        rospy.spin()
    except rospy.ROSInterruptException as ex:
        rospy.logfatal("Fatal ROS error: \"{0}\".".format(ex))
    except Exception as ex:
        rospy.logfatal("Fatal error: \"{0}\".".format(ex))
    except:
        rospy.logfatal("Unknown fatal error.")

if __name__ == '__main__':
    main()