import sys
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotTrajectory
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
        self.group.execute(plan, wait=True)
        self.group.clear_pose_targets()

    def get_current_pose(self):
        return self.group.get_current_pose().pose

    def set_speed_scaling(self, scaling_factor):
        # scaling_factor is a float between 0.0 (no movement) and 1.0 (full speed)
        self.group.set_max_velocity_scaling_factor(scaling_factor)
        self.group.set_max_acceleration_scaling_factor(scaling_factor)

if __name__ == "__main__":
    robot = RobotController("arm_group")
    # pose = Pose()
    # set your pose values
    # robot.move_to_pose(pose)
    print(robot.get_current_pose())
    # robot.set_speed_scaling(0.5)

