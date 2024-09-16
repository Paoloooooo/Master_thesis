from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf.transformations as tft
from math import degrees, radians, sin, cos
from rospy import get_rostime


class Pose:
    """
    A class representing a pose in a 2D plane with a heading (orientation).
    """

    POSEWITHCOVARIANCE_SEQ = 1
    POSE_SEQ = 1

    def __init__(self, x: float, y: float, heading: float):
        """
        Initializes a Pose object with x, y coordinates and a heading.

        Parameters:
        -----------
        x : float
            The x-coordinate of the pose.
        y : float
            The y-coordinate of the pose.
        heading : float
            The heading (orientation) of the pose in radians.
        """
        self._x = x
        self._y = y
        self._heading = heading

    def __str__(self) -> str:
        """
        Returns a human-readable string representation of the Pose object.

        This method is automatically called when the object is printed or converted to a string.
        It provides a clear and concise summary of the Pose object by displaying the x, y,
        and heading attributes, separated by tab spaces.

        Returns:
        --------
        str
            A formatted string containing the x, y coordinates and heading of the Pose object.
        """
        return f"x: {self._x:<3}  y: {self._y:<3}  heading: {self._heading:<3}\t "

    @classmethod
    def posestamped_to_pose(cls, pose_stamped: PoseStamped) -> "Pose":
        """
        Converts a ROS PoseStamped message to a Pose object.

        Parameters:
        -----------
        pose_stamped : PoseStamped
            The ROS PoseStamped message to convert.

        Returns:
        --------
        Pose
            The corresponding Pose object.
        """
        pose = pose_stamped.pose
        x = pose.position.x
        y = pose.position.y

        # Bella la matematica
        quat = pose.orientation
        rot_euler = tft.euler_from_quaternion([quat.w, quat.x, quat.y, quat.z])
        heading = degrees(rot_euler[0]) % 360

        return Pose(x, y, heading)

    @classmethod
    def pose_to_posestamped(cls, pose: "Pose") -> PoseStamped:
        """
        Converts a Pose object to a ROS PoseStamped message.

        Parameters:
        -----------
        pose : Pose
            The Pose object to convert.

        Returns:
        --------
        PoseStamped
            The corresponding ROS PoseStamped message.
        """

        result = PoseStamped()
        result.pose.position.x = pose._x
        result.pose.position.y = pose._y

        result.pose.orientation.z = sin(radians(pose._heading) / 2)
        result.pose.orientation.w = cos(radians(pose._heading) / 2)

        result.header.stamp = get_rostime()
        result.header.frame_id = "map"
        result.header.seq = cls.POSE_SEQ
        cls.POSE_SEQ += 1

        return result

    @classmethod
    def pose_to_posewithcovariancestamped(
        cls, pose: "Pose"
    ) -> PoseWithCovarianceStamped:
        """
        Converts a Pose object to a ROS PoseWithCovarianceStamped message.

        Parameters:
        -----------
        pose : Pose
            The Pose object to convert.

        Returns:
        --------
        PoseWithCovarianceStamped
            The corresponding ROS PoseStamped message.
        """

        result = PoseWithCovarianceStamped()
        result.pose.pose.position.x = pose._x
        result.pose.pose.position.y = pose._y

        result.pose.pose.orientation.z = sin(radians(pose._heading) / 2)
        result.pose.pose.orientation.w = cos(radians(pose._heading) / 2)

        result.header.stamp = get_rostime()
        result.header.frame_id = "map"
        result.header.seq = cls.POSEWITHCOVARIANCE_SEQ
        cls.POSEWITHCOVARIANCE_SEQ += 1

        return result
