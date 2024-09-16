## The constants for the cnavigator node ##
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseActionGoal,
)
from geometry_msgs.msg import PoseWithCovarianceStamped

from rospy import Duration

INITIAL_POSE_TOPIC = "initialpose"
INITIAL_POSE_MESSAGE = PoseWithCovarianceStamped

NAVIGATION_CLIENT_NAME = "move_base"
NAVIGATION_CLIENT_TYPE = MoveBaseAction
NAVIGATION_CLIENT_GOAL = MoveBaseActionGoal()
NAVIGATION_CLIENT_TIMEOUT = Duration(secs=10)
