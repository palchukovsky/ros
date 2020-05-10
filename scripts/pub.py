#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from random import uniform as random

class Pub:

    def __init__(self, topic, number_of_vars):

        self._publisher = rospy.Publisher(
            topic, String, queue_size=number_of_vars)

        if number_of_vars < 1:
            raise Exception('invalid parameter')
        self._messages = []
        for i in range(1, number_of_vars + 1):
            self._messages.append(
                'Publisher text string from slot #{0}.'.format(i))

    def run(self, hz):
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            self._fire()
            rate.sleep()

    def _fire(self):
        self._publisher.publish(self._create_message())

    def _create_message(self):
        i = random(0, len(self._messages) -1 )
        return self._messages[int(i)]

def main():
    try:
        rospy.init_node('palchukovsky_publisher', anonymous=True)
        Pub("/demo/text", 20).run(2)
    except rospy.ROSInterruptException as ex:
        rospy.logfatal('Fatal ROS error: \"{0}".'.format(ex))
    except Exception as ex:
        rospy.logfatal('Fatal error: "{0}".'.format(ex))
    except:
        rospy.logfatal("Unknown fatal error.")

if __name__ == '__main__':
    main()