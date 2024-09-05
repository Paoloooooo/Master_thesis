import rospy
from threading import Thread

from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseFeedback,
    MoveBaseActionGoal,
    MoveBaseActionResult,
)

import actionlib
from actionlib_msgs.msg import GoalStatus

from utils.pose import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped

class Navigator:
    """
    The Navigator class is responsible for managing and executing navigation tasks,
    including setting the initial pose, moving to a specified pose, and checking
    the current position and availability.

    Methods:
    --------
    set_initial_pose(initial_pose: Pose):
        Sets the initial position and orientation of the robot.

    go_to(pose: Pose):
        Directs the robot to move to the specified position and orientation.

    is_free() -> bool:
        Checks if the navigator is currently free to accept new commands.

    get_current_position() -> Pose:
        Retrieves the current position and orientation of the robot.
    """

    def __init__(self):
        """
        Initializes a new instance of the Navigator class.
        """
        rospy.init_node("navigator_node")

        self._current_pose = None
        self._target_pose = None
        self._navigation_completed = True
        self._initial_pose_set = False

        self._navigation_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self._goal = MoveBaseActionGoal()
        connected = self._navigation_client.wait_for_server(
            timeout=rospy.Duration(secs=10)
        )
        if not connected:
            rospy.logfatal("Unable to connect to move base server")
            raise TimeoutError()

        rospy.loginfo("Move base client ready")

        self._initial_pose_publisher = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )

        # Setting amcl params
        """
        bool
        name: "force_update_after_initialpose"
        value: False

        int
        name: "max_particles"
        value: 5000

        double
        name: "transform_tolerance"
        value: 0.1

        """

        rospy.loginfo("Initial pose publisher ready")
        rospy.loginfo("Initialization ended")

    def set_initial_pose(self, initial_pose: Pose):
        """
        Sets the initial position and orientation of the robot.

        Parameters:
        -----------
        initial_pose : Pose
            The initial pose to set.
        """

        self._initial_pose_set = True
        self._current_pose = initial_pose

        # TODO: fix initial pose not being set sometimes
        for _ in range(50):
            msg = Pose.pose_to_posewithcovariancestamped(initial_pose)
            self._initial_pose_publisher.publish(msg)

        rospy.loginfo(f"Set initial pose to: {initial_pose}")

    def go_to(self, pose: Pose):
        """
        Directs the robot to move to the specified position and orientation.

        Parameters:
        -----------
        pose : Pose
            The target pose to move to, typically containing position and orientation data.
        """
        if not self._initial_pose_set:
            rospy.logwarn("Initial pose not set. Navigation may be imprecise.")

        self._navigation_completed = False
        self._goal.goal.target_pose = Pose.pose_to_posestamped(pose)
        
        rospy.loginfo(f"Sending goal: {self._goal}")
        self._navigation_client.send_goal(
            self._goal.goal, feedback_cb=self._got_feedback, done_cb=self._navigation_ended
        )

        rospy.logdebug(f"Starting navigating to: {pose}")

    def is_free(self) -> bool:
        """
        Checks if the navigator is currently free to accept new commands.

        Returns:
        --------
        bool
            True if the navigator is free, False if it is currently engaged in a task.
        """
        return self._navigation_completed

    def get_current_position(self) -> Pose:
        """
        Retrieves the current position and orientation of the robot.

        Returns:
        --------
        Pose
            The current position and orientation of the robot.
        """
        return self._current_pose

    def _got_feedback(self, feedback: MoveBaseFeedback):
        self._current_pose = Pose.posestamped_to_pose(feedback.base_position)

        rospy.logdebug_throttle(
            0.5, f"Received feedback: {feedback}"
        )

    def _navigation_ended(self, status: GoalStatus, result: MoveBaseActionResult):
        self._navigation_completed = True

        rospy.loginfo(f'{status}\n{result}')

        return
        # TODO: status management
        """             uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server """

        if nav_state == GoalStatus.SUCCEEDED:
            self._navigation_ended = True
            rospy.loginfo("Navigation to target pose completed.")
        elif nav_state == GoalStatus.ABORTED or nav_state == GoalStatus.REJECTED:
            self._navigation_ended = True
            rospy.logerr("Navigation task ended anomalously.")
