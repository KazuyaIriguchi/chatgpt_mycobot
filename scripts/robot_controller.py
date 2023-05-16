import sys
import rospy
import tf
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose

class RobotController:
    def __init__(self, group_name):
        roscpp_initialize(sys.argv)
        rospy.init_node('moveit_node')

        self.group = MoveGroupCommander(group_name)

    def move_to_pose(self, target_pose):
        # target_pose is a geometry_msgs.msg.Pose instance
        self.group.set_pose_target(target_pose)
        plan = self.group.plan()
        self.group.execute(plan[1], wait=True)
        self.group.clear_pose_targets()

    def get_current_pose(self):
        return self.group.get_current_pose().pose

    def set_speed_scaling(self, scaling_factor):
        # scaling_factor is a float between 0.0 (no movement) and 1.0 (full speed)
        self.group.set_max_velocity_scaling_factor(scaling_factor)
        self.group.set_max_acceleration_scaling_factor(scaling_factor)

    def list_to_pose(self, pose_list):
        # pose_list is a list in the format [x, y, z, rx, ry, rz]
        pose = Pose()
        pose.position.x = pose_list[0]
        pose.position.y = pose_list[1]
        pose.position.z = pose_list[2]
        quaternion = tf.transformations.quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def move_to_pose_list(self, pose_list):
        pose = self.list_to_pose(pose_list)
        self.move_to_pose(pose)

if __name__ == "__main__":
    robot = RobotController("arm_group")
    pose_list = [0.132, -0.150, 0.075, 3.140, 0.027, 3.089]  # Example list
    robot.move_to_pose_list(pose_list)
    print(robot.get_current_pose())
    robot.set_speed_scaling(0.5)

