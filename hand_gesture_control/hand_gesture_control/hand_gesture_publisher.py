import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool 
# from hand_gesture_control.msg import ArmCommand

import cv2
import numpy as np
import mediapipe as mp

XY_SCALE = 0.5
YAW_SMOOTHING = 0.5
GRIPPER_THRESHOLD = 0.05
yaw_history = {'Left': 0.0, 'Right': 0.0}

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

class HandGesturePublisher(Node):
    def __init__(self):
        super().__init__('hand_gesture_publisher')

        # seperate version #
        self.left_pose_pub = self.create_publisher(Pose2D, '/left_arm/target_pose', 10)
        self.right_pose_pub = self.create_publisher(Pose2D, '/right_arm/target_pose', 10)
        self.left_gripper_pub = self.create_publisher(Bool, '/left_arm/gripper_open', 10)
        self.right_gripper_pub = self.create_publisher(Bool, '/right_arm/gripper_open', 10)

        # target_pose(Pose2D) and gripper_open(Bool) in ArmCommand #
        # self.left_pub = self.create_publisher(ArmCommand, 'left_arm/command', 10)
        # self.right_pub = self.create_publisher(ArmCommand, 'right_arm/command', 10)

        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.03, self.timer_callback)

        self.get_logger().info('Hand gesture ROS2 publisher started.')

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)

        if results.multi_hand_landmarks and results.multi_handedness:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                label = results.multi_handedness[i].classification[0].label
                landmarks = hand_landmarks.landmark

                wrist = landmarks[0]
                index_tip = landmarks[8]
                middle_tip = landmarks[12]
                thumb_tip = landmarks[4]
                ring_tip = landmarks[16]

                x_norm = (wrist.x - 0.5) * 2.0
                y_norm = (0.5 - wrist.y) * 2.0
                x = x_norm * XY_SCALE
                y = y_norm * XY_SCALE

                dx = middle_tip.x - index_tip.x
                dy = middle_tip.y - index_tip.y
                raw_yaw = np.arctan2(dy, dx)
                yaw = (1 - YAW_SMOOTHING) * yaw_history[label] + YAW_SMOOTHING * raw_yaw
                yaw_history[label] = yaw

                dist = np.linalg.norm(np.array([thumb_tip.x - ring_tip.x, thumb_tip.y - ring_tip.y]))
                is_open = dist > GRIPPER_THRESHOLD

                # print information
                # print(f"[{label} hand] pose=({x:.2f}, {y:.2f}, {yaw:.2f}), gripper={'OPEN' if is_open else 'CLOSED'}")

                # seperate version #
                pose_msg = Pose2D()
                pose_msg.x = x
                pose_msg.y = y
                pose_msg.theta = yaw

                grip_msg = Bool()
                grip_msg.data = bool(is_open)

                if label == 'Left':
                    self.left_pose_pub.publish(pose_msg)
                    self.left_gripper_pub.publish(grip_msg)
                elif label == 'Right':
                    self.right_pose_pub.publish(pose_msg)
                    self.right_gripper_pub.publish(grip_msg)

                
                # Use ArmCommand #
                # msg = ArmCommand()
                # msg.pose.x = x
                # msg.pose.y = y
                # msg.pose.theta = yaw
                # msg.gripper_open = bool(is_open)

                # if label == 'Left':
                #     self.left_pub.publish(msg)
                # elif label == 'Right':
                #     self.right_pub.publish(msg)

                # draw inportant points of hands
                mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style()
                )

                wrist_px = int(wrist.x * w)
                wrist_py = int(wrist.y * h)
                cv2.putText(frame, f"{label} Hand", (wrist_px, wrist_py - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"x={x:.2f} y={y:.2f} yaw={yaw:.2f} open={is_open}",
                            (wrist_px, wrist_py + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)    

        cv2.imshow('Hand Gesture Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandGesturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
