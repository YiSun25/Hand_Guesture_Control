
###############################################################
#          First                               #
################################################################

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Bool
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, RobotState
from moveit_msgs.msg import PlanningOptions
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive
import math
import time


class DualArmGestureController(Node):
    def __init__(self):
        super().__init__('hand_gesture_subscriber')

        self.left_pose = []
        self.right_pose = []
        self.left_gripper = []
        self.right_gripper = []

        # Create action clients for both arms
        self.left_arm_client = ActionClient(self, MoveGroup, 'move_action')
        self.right_arm_client = ActionClient(self, MoveGroup, 'move_action')

        # Gripper joint names
        self.left_gripper_joint = 'left_gripper_robotiq_85_left_knuckle_joint'
        self.right_gripper_joint = 'right_gripper_robotiq_85_left_knuckle_joint'

        # Subscriptions
        self.create_subscription(Pose2D, '/left_arm/target_pose', self.left_pose_cb, 10) # left_pose_callback
        self.create_subscription(Pose2D, '/right_arm/target_pose', self.right_pose_cb, 10)
        self.create_subscription(Bool, '/left_arm/gripper_open', self.left_gripper_cb, 10)
        self.create_subscription(Bool, '/right_arm/gripper_open', self.right_gripper_cb, 10)

        self.get_logger().info('Gesture-based dual-arm controller started.')
        self.loop()

    def create_goal(self, pose: PoseStamped, group_name: str):
        req = MotionPlanRequest()
        req.group_name = group_name
        req.goal_constraints.append(Constraints())
        req.goal_constraints[0].position_constraints.append(PositionConstraint())
        req.goal_constraints[0].position_constraints[0].constraint_region.primitives.append(SolidPrimitive())
        req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions = [0.001]
        req.goal_constraints[0].position_constraints[0].target_point_offset.x = 0.0
        req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.append(pose.pose)
        req.goal_constraints[0].position_constraints[0].header = pose.header

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        return goal

    def pose2d_to_pose_stamped(self, msg: Pose2D, frame: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = -1.
        pose.pose.position.y = 0.
        pose.pose.position.z = 0.5

        # Yaw only, convert to quaternion
        yaw = msg.theta
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def left_pose_cb(self, msg: Pose2D):
        pose = self.pose2d_to_pose_stamped(msg, 'ur5_left_base_link')
        goal = self.create_goal(pose, 'left_arm')
        self.get_logger().info('++++++++++++++++++++++++++++++++++')
        self.left_pose.append(goal)
        
        # self.left_arm_client.wait_for_server()
        # self.left_arm_client.send_goal_async(goal)

    def right_pose_cb(self, msg: Pose2D):
        pose = self.pose2d_to_pose_stamped(msg, 'ur5_right_base_link')
        goal = self.create_goal(pose, 'right_arm')
        # self.right_arm_client.wait_for_server()
        # self.right_arm_client.send_goal_async(goal)

    def left_gripper_cb(self, msg: Bool):
        joint = JointState()
        joint.name = [self.left_gripper_joint]
        joint.position = [0.8 if msg.data else 0.0]
        joint.header.stamp = self.get_clock().now().to_msg()
        # You may need a publisher to /left_gripper_controller/command

    def right_gripper_cb(self, msg: Bool):
        joint = JointState()
        joint.name = [self.right_gripper_joint]
        joint.position = [0.8 if msg.data else 0.0]
        joint.header.stamp = self.get_clock().now().to_msg()
        # You may need a publisher to /right_gripper_controller/command

    def send_pose(self):
        self.left_arm_client.wait_for_server()
        
        goal = []
        if not len(self.left_pose) == 0:
            print('**************************************')
            goal = self.left_pose[0]
            self.left_pose.pop(0)
            self.left_arm_client.send_goal_async(goal)
    
    def loop(self):
        while rclpy.ok() :
            rclpy.spin_once(self)
            self.send_pose()
            time.sleep(0.3)
            


def main(args=None):
    rclpy.init(args=args)
    node = DualArmGestureController()
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


###################################################################
# Second  
#################################################################
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose2D
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseStamped
# import math
# import tf_transformations

# from moveit2 import MoveIt2
# # from moveit_commander import MoveGroupCommander
# # from moveit_ros2_planning_interface_py.planning_interface import MoveGroupInterface



# class DualArmGestureController(Node):
#     def __init__(self):
#         super().__init__('hand_gesture_subscriber')

#         # Initialize MoveIt Controller
#         self.left_arm = MoveIt2(
#             node=self,
#             joint_names=[
#                 'ur5_left_shoulder_pan_joint',
#                 'ur5_left_shoulder_lift_joint',
#                 'ur5_left_elbow_joint',
#                 'ur5_left_wrist_1_joint',
#                 'ur5_left_wrist_2_joint',
#                 'ur5_left_wrist_3_joint',
#             ],
#             base_link_name='ur5_left_base_link',
#             end_effector_name='ur5_left_tool0',
#             group_name='left_arm',
#             execute_via_moveit=True
#         )

#         self.right_arm = MoveIt2(
#             node=self,
#             joint_names=[
#                 'ur5_right_shoulder_pan_joint',
#                 'ur5_right_shoulder_lift_joint',
#                 'ur5_right_elbow_joint',
#                 'ur5_right_wrist_1_joint',
#                 'ur5_right_wrist_2_joint',
#                 'ur5_right_wrist_3_joint',
#             ],
#             base_link_name='ur5_right_base_link',
#             end_effector_name='ur5_right_tool0',
#             group_name='right_arm',
#             execute_via_moveit=True
#         )

#         # gripper joint name
#         self.left_gripper_joint = 'left_gripper_robotiq_85_left_knuckle_joint'
#         self.right_gripper_joint = 'right_gripper_robotiq_85_left_knuckle_joint'

#         # subscribe hand gesture info
#         self.create_subscription(Pose2D, '/left_arm/target_pose', self.left_pose_cb, 10)
#         self.create_subscription(Pose2D, '/right_arm/target_pose', self.right_pose_cb, 10)
#         self.create_subscription(Bool, '/left_arm/gripper_open', self.left_gripper_cb, 10)
#         self.create_subscription(Bool, '/right_arm/gripper_open', self.right_gripper_cb, 10)

#         self.get_logger().info('Dual-arm gesture controller started.')

#     def pose2d_to_pose_stamped(self, msg: Pose2D, frame: str) -> PoseStamped:
#         pose = PoseStamped()
#         pose.header.frame_id = frame
#         pose.pose.position.x = 0.4 + msg.x
#         pose.pose.position.y = msg.y
#         pose.pose.position.z = 0.3

#         # yaw to quaternion
#         q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
#         pose.pose.orientation.x = q[0]
#         pose.pose.orientation.y = q[1]
#         pose.pose.orientation.z = q[2]
#         pose.pose.orientation.w = q[3]
#         return pose

#     def left_pose_cb(self, msg: Pose2D): 
#         pose = self.pose2d_to_pose_stamped(msg, 'ur5_left_base_link')
#         self.left_arm.move_to_pose(pose)

#     def right_pose_cb(self, msg: Pose2D):
#         pose = self.pose2d_to_pose_stamped(msg, 'ur5_right_base_link')
#         self.right_arm.move_to_pose(pose)

#     def left_gripper_cb(self, msg: Bool):
#         pos = 0.8 if msg.data else 0.0
#         self.left_arm.move_to_configuration({self.left_gripper_joint: pos})

#     def right_gripper_cb(self, msg: Bool):
#         pos = 0.8 if msg.data else 0.0
#         self.right_arm.move_to_configuration({self.right_gripper_joint: pos})


# def main(args=None):
#     rclpy.init(args=args)
#     node = DualArmGestureController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
