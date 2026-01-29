import threading
import time
from typing import Dict

import numpy as np
import cv2
import rclpy
from rclpy.node import Node as ROS2Node
from sensor_msgs.msg import JointState, Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


import logging_mp

CONNECT_TIMEOUT_FRAME = 10
logger = logging_mp.get_logger(__name__)

class LEJUKuavoRos2Node(ROS2Node):
    def __init__(self):
        super().__init__('ros2_recv_pub_driver')
        self.stop_spin = False  # 初始化停止标志
        self.qos = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.qos_best_effort = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 统一QoS配置（修复原问题）
        self.publisher_left_arm = self.create_publisher(
            JointState, "/kuavo_arm_traj", self.qos
        )
        self.publisher_right_arm = self.create_publisher(
            JointState, "/kuavo_arm_traj", self.qos
        )
        self.publisher_left_hand = self.create_publisher(
            robotHandPosition, "/control_robot_hand_position", self.qos  
        )
        self.publisher_right_hand = self.create_publisher(
            robotHandPosition, "/control_robot_hand_position", self.qos  
        )
        self.publisher_state_head = self.create_publisher(
            robotHeadMotionData, "/robot_head_motion_data", self.qos
        )
     
        self.last_main_send_time_ns = 0
        self.last_follow_send_time_ns = 0
        self.min_interval_ns = 1e9 / 30  # 30Hz

        self._init_message_main_filters()
        self._init_message_follow_filters()
        self._init_image_message_filters()

        self.recv_images: Dict[str, float] = {}
        self.recv_leader: Dict[str, float] = {}
        self.recv_follower: Dict[str, float] = {}
        self.recv_images_status: Dict[str, int] = {}
        self.recv_leader_status: Dict[str, int] = {}
        self.recv_follower_status: Dict[str, int] = {}

        self.lock = threading.Lock()

    def _init_message_follow_filters(self):
           self.joint_states_sub = self.create_subscription(
             sensorsData,
             '/sensors_data_raw',
             self.joint_states_callback,
             10
            )
           self.get_logger().info('已订阅 /sensors_data_raw 话题')
       
 
    def joint_states_callback(self, msg):
        """关节状态回调函数"""
        try:
            # 频率控制：检查是否达到最小时间间隔
            current_time_ns = time.time_ns()
            if (current_time_ns - self.last_follow_send_time_ns) < self.min_interval_ns:
                return  # 距离上次更新时间太短，丢弃本次数据
            self.last_follow_send_time_ns = current_time_ns
 
            # 提取所有关节位置数据
            joint_positions = np.array(msg.position, dtype=np.float32)
            
            # 第一次接收到数据时打印日志
            if 'follower_arms' not in self.recv_follower:
                self.get_logger().info(f'首次接收到关节数据: {len(joint_positions)} 个关节')
                self.get_logger().info(f'关节名称: {msg.name}')
            
            # 线程安全地更新共享数据
            with self.lock:
                self.recv_follower['follower_arms'] = joint_positions
                # 重置连接状态计数器
                self.recv_follower_status['follower_arms'] = CONNECT_TIMEOUT_FRAME
        except Exception as e:
            self.get_logger().error(f"关节状态回调错误: {e}")
           

    #def _init_message_main_filters(self):
        #sub_joint_left = Subscriber(self, JointState, '/motion_target/target_joint_state_arm_left', qos_profile=self.qos_best_effort)
        #sub_joint_right = Subscriber(self, JointState, '/motion_target/target_joint_state_arm_right', qos_profile=self.qos_best_effort)
        #sub_joint_torso = Subscriber(self, JointState, '/motion_target/target_joint_state_torso', qos_profile=self.qos_best_effort)
        #sub_pose_left = Subscriber(self, PoseStamped, '/motion_target/target_pose_arm_left', qos_profile=self.qos_best_effort)
       # sub_pose_right = Subscriber(self, PoseStamped, '/motion_target/target_pose_arm_right', qos_profile=self.qos_best_effort)
       # sub_torso = Subscriber(self, PoseStamped, '/motion_target/target_pose_torso', qos_profile=self.qos_best_effort)
       # sub_gripper_left = Subscriber(self, JointState, '/motion_target/target_position_gripper_left', qos_profile=self.qos_best_effort)
       # sub_gripper_right = Subscriber(self, JointState, '/motion_target/target_position_gripper_right', qos_profile=self.qos_best_effort)

       # self.pose_sync = ApproximateTimeSynchronizer(
       #     [sub_joint_left, sub_joint_right, sub_joint_torso, sub_pose_left, sub_pose_right, sub_torso, sub_gripper_left, sub_gripper_right],
       #     queue_size=50,
       #     slop=0.01
       # )
       # self.pose_sync.registerCallback(self.synchronized_main_callback)
 
   # def synchronized_main_callback(self, joint_left, joint_right, joint_torso, pose_left, pose_right, torso, gripper_left, gripper_right):
        #try:
         #   current_time_ns = time.time_ns()
          #  if (current_time_ns - self.last_main_send_time_ns) < self.min_interval_ns:
          #      return
         #   self.last_main_send_time_ns = current_time_ns

          #  left_joint = np.array(joint_left.position, dtype=np.float32)
          #  right_joint = np.array(joint_right.position, dtype=np.float32)
 
         #   gripper_left_pose = np.array(gripper_left.position, dtype=np.float32)
          #  gripper_right_pose = np.array(gripper_right.position, dtype=np.float32)
          #  torso_joint = np.array(joint_torso.position, dtype=np.float32)

            #left_pos = np.array([
           #     pose_left.pose.position.x, pose_left.pose.position.y, pose_left.pose.position.z,
           #     pose_left.pose.orientation.x, pose_left.pose.orientation.y, pose_left.pose.orientation.z, pose_left.pose.orientation.w
           # ], dtype=np.float32)

          #  right_pos = np.array([
           #     pose_right.pose.position.x, pose_right.pose.position.y, pose_right.pose.position.z,
           #     pose_right.pose.orientation.x, pose_right.pose.orientation.y, pose_right.pose.orientation.z, pose_right.pose.orientation.w
           # ], dtype=np.float32)

           # torso_pose = np.array([
           #     torso.pose.position.x, torso.pose.position.y, torso.pose.position.z,
           #     torso.pose.orientation.x, torso.pose.orientation.y, torso.pose.orientation.z, torso.pose.orientation.w
           # ], dtype=np.float32)

          #  merged_data = np.concatenate([left_joint, gripper_left_pose, right_joint, gripper_right_pose, torso_joint, left_pos, right_pos, torso_pose])
           # with self.lock:
         ##       self.recv_leader['leader_arms'] = merged_data
          #      self.recv_leader_status['leader_arms'] = CONNECT_TIMEOUT_FRAME
       # except Exception as e:
        #    self.get_logger().error(f"Pose callback error: {e}")

    def _init_image_message_filters(self):
        sub_camera_top = Subscriber(self, Image, '')
        sub_camera_wrist_left = Subscriber(self, Image, '')
        sub_camera_wrist_right = Subscriber(self, Image, '')
 
        self.image_sync = ApproximateTimeSynchronizer(
            [sub_camera_top, sub_camera_wrist_left, sub_camera_wrist_right],
            queue_size=5,
            slop=0.1
        )
        self.image_sync.registerCallback(self.image_synchronized_callback)

    def image_synchronized_callback(self, top, wrist_left, wrist_right):
        try:
            self.images_recv(top, "image_top", 1280, 720)
            self.images_recv(wrist_left, "image_wrist_left", 640, 360)
            self.images_recv(wrist_right, "image_wrist_right", 640, 360)
        except Exception as e:
            self.get_logger().error(f"Image synchronized callback error: {e}")
    
    def images_recv(self, msg, event_id, width, height, encoding="jpeg"):
        try:
            if 'image' in event_id:
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                if encoding == "bgr8":
                    channels = 3
                    frame = img_array.reshape((height, width, channels)).copy()
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                elif encoding == "rgb8":
                    channels = 3
                    frame = img_array.reshape((height, width, channels))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    channels = 3
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                elif encoding == "depth16":
                    frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width,1)
                
                if frame is not None:
                    with self.lock:
                        self.recv_images[event_id] = frame
                        self.recv_images_status[event_id] = CONNECT_TIMEOUT_FRAME
        except Exception as e:
            logger.error(f"recv image error: {e}")

    def ros_replay(self, array):
        try:
            def normalize_precision(val, decimals=3):
                val = float(val)
                if np.isnan(val) or np.isinf(val):
                    self.get_logger().warning(f"检测到非法值 {val}，替换为0.0")
                    return 0.0
                return round(val, decimals)
            
            left_arm = [normalize_precision(v) for v in array[:6]]
            left_gripper = [normalize_precision(v) for v in array[6:7]]
            right_arm = [normalize_precision(v) for v in array[7:13]]
            right_gripper = [normalize_precision(v) for v in array[13:14]]
            msg = JointState()
            msg.position = left_arm 
            self.publisher_left_arm.publish(msg)

            msg = JointState()
            msg.position = right_arm
            self.publisher_right_arm.publish(msg)

            msg = JointState()
            msg.position = left_gripper
            self.publisher_left_gripper.publish(msg)

            msg = JointState()
            msg.position = right_gripper
            self.publisher_right_gripper.publish(msg)

            msg = JointState()
            msg.position = torso  
            self.publisher_state_torso.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error during replay at frame: {e}")
            raise

    def destroy(self):
        self.stop_spin = True
        super().destroy_node()

   

# 保留ros_spin_thread函数（供外部调用）
def ros_spin_thread(node):
    while rclpy.ok() and not getattr(node, "stop_spin", False):
        rclpy.spin_once(node, timeout_sec=0.01)