#!/usr/bin/env python

import rospy
from constants.costmap_clearer import RATE, SERVICE_NAME, SERVICE_REQUEST, SERVICE_TYPE


class Clearer:
    # TODO: implement this -> rospy.Rate

    def __init__(self):
        rospy.init_node("costmap_clearer")

        self._client = rospy.ServiceProxy(SERVICE_NAME, SERVICE_TYPE)
        self._rate = rospy.Rate(RATE)

    def loop(self):
        while True:
            rospy.wait_for_service(SERVICE_NAME)
            rospy.loginfo("Clearing costmap")
            self._client.call(SERVICE_REQUEST)
            self._rate.sleep()


if __name__ == "__main__":
    c = Clearer()

    try:
        c.loop()
    except Exception as e:
        rospy.loginfo(f"Clearer stopped due to the Exception {e}")
