
###############################################################
#          First                               #
################################################################

import moveit_msgs.action
import moveit_msgs.msg
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Bool
from moveit_msgs.action import MoveGroup, ExecuteTrajectory 
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, RobotState, OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive
import math
import time
from pymoveit2 import MoveIt2

class DualArmGestureController(Node):
    def __init__(self):
        super().__init__('hand_gesture_subscriber')
        self.left_pose = []
        self.right_pose = []
        self.left_goal = []
        self.right_goal = []
        self.left_gripper = []
        self.right_gripper = [] 
        self.moveit = MoveIt2(node=self, 
                              joint_names=[
                                           'ur5_left_shoulder_pan_joint',
                                           'ur5_left_shoulder_lift_joint',
                                           'ur5_left_elbow_joint',
                                           'ur5_left_wrist_1_joint',
                                           'ur5_left_wrist_2_joint',
                                           'ur5_left_wrist_3_joint',
                                        #    'left_gripper_robotiq_85_left_knuckle_joint'
                                           ],
                              base_link_name='ur5_left_base_link',
                              end_effector_name='ur5_left_tool0',
                              group_name='left_arm',
                              execute_via_moveit=False
                              ) 
        self.moveit.allowed_planning_time = 10.
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

    def create_goal(self, pose: PoseStamped, group_name: str, link: str):
        # req = MotionPlanRequest()
        # req.group_name = group_name
        # req.goal_constraints.append(Constraints())
        # req.goal_constraints[0].position_constraints.append(PositionConstraint())
        # req.goal_constraints[0].position_constraints[0].constraint_region.primitives.append(SolidPrimitive())
        # req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        # req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions = [0.001]
        # req.goal_constraints[0].position_constraints[0].target_point_offset.x = 0.0
        # req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.append(pose.pose)
        # req.goal_constraints[0].position_constraints[0].header = pose.header
        

        # goal = MoveGroup.Goal()
        # goal.request = req
        # goal.planning_options = PlanningOptions()
        # goal.planning_options.plan_only = False
        # goal.planning_options.look_around = False
        # goal.planning_options.replan = True

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.1]  # 半径为 1mm 的小球

        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        print(pose.header.frame_id)
        position_constraint.link_name = link
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)
        position_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = link
        ori_constraint.orientation = pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0


        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        req = MotionPlanRequest()
        req.group_name = group_name
        req.goal_constraints.append(constraints)
        req.start_state.joint_state.position = [0.0, -1.5, 0., -1.5, 0.0, 1.5]

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
        # pose.header.frame_id = "world"       
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.
        pose.pose.position.z = 0.5

        # Yaw only, convert to quaternion
        yaw = msg.theta
        # pose.pose.orientation.z = math.sin(yaw / 2.0)
        # pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.orientation.w = 1.
        pose.pose.orientation.x = 0.
        pose.pose.orientation.y = 0.
        pose.pose.orientation.z = 0.
        return pose
    

    def left_pose_cb(self, msg: Pose2D):
        pose = self.pose2d_to_pose_stamped(msg, 'ur5_left_base_link')
        goal = self.create_goal(pose, 'left_arm', 'ur5_left_tool0')
        self.left_goal.append(goal)
        self.left_pose.append(pose)
        
        # self.left_arm_client.wait_for_server()
        # self.left_arm_client.send_goal_async(goal)

    def right_pose_cb(self, msg: Pose2D):
        pose = self.pose2d_to_pose_stamped(msg, 'ur5_right_base_link')
        goal = self.create_goal(pose, 'right_arm', 'ur5_right_tool0')
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

    def moveit2_send(self):
        if not len(self.left_pose) == 0:
            pose = self.left_pose[0]
            self.left_pose.pop(0)
            # self.moveit.set_pose_goal(pose=pose.pose)
            MoveIt2.compute_ik
            print('====================')
            self.moveit.move_to_pose(pose, target_link='ur5_left_base_link', cartesian=True, cartesian_max_step=0.3)
        
    def send_pose(self):
        
        goal = []
        if not len(self.left_goal) == 0:
            goal = self.left_goal[0]
            self.left_goal.pop(0)
            self.left_arm_client.wait_for_server()
            send_goal_future = self.left_arm_client.send_goal_async(goal)

            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                print("Goal was rejected by the server.")
                exit()

            # 获取规划和执行结果
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            # 输出执行结果
            print("Planning result error code:", result.result.error_code.val)
    
    def loop(self):
        while rclpy.ok() :
            rclpy.spin_once(self)
            # self.send_pose()
            self.moveit2_send()
            time.sleep(1.0)
            


def main(args=None):
    rclpy.init(args=args)
    node = DualArmGestureController()
    rclpy.spin(node)
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
