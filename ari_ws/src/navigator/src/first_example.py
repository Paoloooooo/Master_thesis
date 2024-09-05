#!/usr/bin/env python

from utils.navigator import Navigator
from rospy import sleep, Duration
from utils.pose import Pose

"""

      x: 2.206923007965088
      y: -6.921241283416748

      x: -1.7008137702941895
      y: -5.3305983543396

      x: -1.8120901584625244
      y: -0.03878498077392578
      

"""
WAYPOINTS = [Pose(2.0, -6.0, 0.0), Pose(-1.5, -5.0, 180.0), Pose(-2.0, 0.0, -90.0)]
INITIAL_POSE = Pose(0.5, -11.0, 53.0)


def main():
    nav = Navigator()
    nav.set_initial_pose(INITIAL_POSE)

    for wpt in WAYPOINTS:
        input("Go to the next waypoint?")
        nav.go_to(wpt)

        while not nav.is_free():
            curr_pos = nav.get_current_position()

            print(f"Now I'm here:{curr_pos}")
            sleep(Duration(secs=5))


if __name__ == "__main__":
    main()
