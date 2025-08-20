#!/usr/bin/env python3



# *** this is the control scrtipt for the joint group position controller ***
# import math
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState

# class SimpleTest(Node):
#     def __init__(self):
#         super().__init__('simple_test')
        
#         # Publisher for commands
#         self.pub = self.create_publisher(
#             Float64MultiArray, 
#             '/joint_group_position_controller/commands',
#             10
#         )
        
#         # Subscriber to monitor joint states
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )
        
#         # Create timer for periodic publishing
#         self.timer = self.create_timer(0.1, self.publish_command)
#         self.count = 0
        
#     def joint_state_callback(self, msg):
#         # Print current joint positions
#         self.get_logger().info(f'Current joint positions: {[round(p,3) for p in msg.position]}')
        
#     def publish_command(self):
#         msg = Float64MultiArray()
#         msg.data = [
#             math.radians(180),   # joint_base 
#             math.radians(0),   # joint_base_big_arm
#             math.radians(0),   # joint_big_arm_small_arm 
#             math.radians(0),   # joint_small_arm_wrist
#             math.radians(0),   # joint_wrist_hand
#         ]
        
#         self.pub.publish(msg)
#         self.get_logger().info(f'Published command: {[round(math.degrees(x),3) for x in msg.data]}')
        
#         self.count += 1
#         if self.count >= 1:  # Run for ~5 seconds
#             self.get_logger().info('Shutting down...')
#             self.timer.cancel()
#             rclpy.shutdown()

# def main():
#     rclpy.init()
#     node = SimpleTest()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# *** this is the control script for the joint position controller ***
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JTCOnce(Node):
    def __init__(self):
        super().__init__('jtc_once')

        # === 根据你的控制器名字修改 ===
        self.controller_name = 'arm_controller'
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.controller_name}/follow_joint_trajectory'
        )

        # === 按 URDF 顺序填写关节名 ===
        self.joint_names = [
            'joint_base',
            'joint_base_big_arm',
            'joint_big_arm_small_arm',
            'joint_small_arm_wrist',
            'joint_wrist_hand'
        ]

        # 等待 JTC action server
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('JTC action server 未就绪（检查控制器是否已 spawn）')
            rclpy.shutdown()
            return

        # 只发一个目标点：关节1到 180deg，其它 0deg，用 3s 到位
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(180), 0.0, 0.0, 0.0, 0.0]
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1)  # 允许收敛余量，可选

        self.get_logger().info('发送轨迹目标…')
        send_future = self.client.send_goal_async(goal, feedback_callback=self._fb)
        send_future.add_done_callback(self._on_goal_response)

    def _fb(self, feedback):
        # 反馈里有 desired/actual/error，可选打印一部分
        try:
            a0 = feedback.feedback.actual.positions[0]
            self.get_logger().info(f'反馈 actual[0]={a0:.3f} rad')
        except Exception:
            pass

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            rclpy.shutdown()
            return
        self.get_logger().info('目标已接受，等待执行结果…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f'执行完成，error_code={result.error_code}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = JTCOnce()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
