#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import Twist
import math
from turtlesim.msg import Pose
from c_interface.action import Navigate , Square
from c_interface.srv import Setspeed, Clearc
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen
from rclpy.callback_groups import ReentrantCallbackGroup


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        cb = ReentrantCallbackGroup()
        self.current_pose = None
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value


        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose1_callback, 10,
            callback_group=cb
        )
        self._action_server = ActionServer(
            self,
            Navigate,
            'go_to_pose',
            self.pose_callback,
            callback_group=cb
        )

        self._go_to_pose_client = ActionClient(
            self,
            Navigate,
            'go_to_pose'
        )

        self._action_server_square = ActionServer(
            self,
            Square,
            'draw_square',
            self.square_callback,
            callback_group=cb
        )


        self.srv = self.create_service(
            Setspeed, 
            'set_speed_limits',
            self.set_speeds_callback,
            callback_group=cb
        )

        self.srv_reset = self.create_service(
            Clearc,
            'reset_turtle',
            self.reset_turtle,
            callback_group=cb

        )
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=cb)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen', callback_group=cb)
        self.clear_client = self.create_client(Empty, '/clear', callback_group=cb)

        self.get_logger().info('pkg1 turtle_controller node is active with parameters and service.')

    def set_speeds_callback(self, request, response):
        try:

            self.max_linear = request.linear_limit
            self.max_angular = request.angular_limit
            if((self.max_linear > 0) and (self.max_angular > 0)):
                # Sync updates back to the ROS 2 parameter server
                self.set_parameters([
                    rclpy.parameter.Parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE, self.max_linear),
                    rclpy.parameter.Parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE, self.max_angular)
                ])

                response.success = True
                response.message = 'Changed Successfully'
                self.get_logger().info(f'Successfully updated speeds -> Linear: {self.max_linear}, Angular: {self.max_angular}')
            else:
                response.success = False
                response.message = 'Failed: Invalid Input'
                self.get_logger().info(f'Failed to update speeds: invaild input')

        
        except Exception as e:
            response.success = False
            response.message = 'Falied to change'
            self.get_logger().error(f'Failed to update speeds: {e}')

        return response

    def pose1_callback(self,msg):
        self.current_pose = msg

    def normalize_angle(self, angle):
        # Helper to wrap angles between -pi and pi
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def pose_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: x={goal_handle.request.x}, y={goal_handle.request.y}, yaw={goal_handle.request.yaw}')

        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()
        
        # Tolerances
        distance_tolerance = 0.05
        angle_tolerance = 0.01

        # Control gains
        k_linear = 1.0
        k_angular = 4.0

        rate = self.create_rate(20) 
        while rclpy.ok():
            if self.current_pose is None:
                rate.sleep()
                continue
            # 6. Read max speeds dynamically every iteration so changes take effect live
            max_linear = self.get_parameter('max_linear_speed').value
            max_angular = self.get_parameter('max_angular_speed').value

            dx = goal_handle.request.x - self.current_pose.x
            dy = goal_handle.request.y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # 5. Publish feedback while moving
            feedback_msg.remaining_distance = distance
            goal_handle.publish_feedback(feedback_msg)

            twist = Twist()

            if distance > distance_tolerance:
                angle_to_goal = math.atan2(dy, dx)
                heading_error = self.normalize_angle(angle_to_goal - self.current_pose.theta)


                linear_speed = min(max_linear, k_linear * distance)
                angular_speed = min(max_angular, max(-max_angular, k_angular * heading_error))

                # If facing away, prioritize turning
                if abs(heading_error) > math.pi / 2:
                    linear_speed = 0.0

                twist.linear.x = linear_speed
                twist.angular.z = angular_speed
            else:
                # 4. Rotate to the final yaw at the end
                yaw_error = self.normalize_angle(goal_handle.request.yaw - self.current_pose.theta)
                
                if abs(yaw_error) > angle_tolerance:
                    twist.linear.x = 0.0
                    twist.angular.z = min(max_angular, max(-max_angular, k_angular * yaw_error))
                else:
                    # Goal reached completely!
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    break


            self.publisher_.publish(twist)
            rate.sleep()


        self.publisher_.publish(Twist())

        result.success = True
        result.final_x = self.current_pose.x
        result.final_y = self.current_pose.y
        result.final_theta = self.current_pose.theta

        goal_handle.succeed()
        return result

    async def set_pen(self, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = 179, 184, 255, 3  # turtlesim defaults
        req.off = 1 if off else 0
        if not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SetPen service not available')
            return None
        return await self.set_pen_client.call_async(req)

    async def send_go_to_pose(self, x, y, yaw):
        if not self._go_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('go_to_pose action server not available')
            return None

        goal_msg = Navigate.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.yaw = float(yaw)

        send_goal_future = self._go_to_pose_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().warn('go_to_pose goal was rejected')
            return None

        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.result

    async def reset_turtle(self, request, response):
        self.get_logger().info('Reset service triggered: Resetting turtle position and clearing canvas...')
        try:
            if not self.teleport_client.wait_for_service(timeout_sec=1.0):
                response.success = False
                response.message = 'Teleport service unavailable'
                self.get_logger().warn('TeleportAbsolute service not available')
                return response

            if not self.clear_client.wait_for_service(timeout_sec=1.0):
                response.success = False
                response.message = 'Clear service unavailable'
                self.get_logger().warn('Clear service not available')
                return response

            await self.set_pen(True)                                # pen up so no line is drawn
            
            tele = TeleportAbsolute.Request()
            tele.x, tele.y, tele.theta = 5.5, 5.5, 0.0
            await self.teleport_client.call_async(tele)             # back to start
            
            await self.clear_client.call_async(Empty.Request())     # wipe the screen
            
            await self.set_pen(False)                               # pen down again
            response.success = True
            self.get_logger().info('Turtle successfully reset to home position (5.5, 5.5, 0.0).')
        except Exception as e:
            response.success = False
            self.get_logger().error(f'Reset error: {e}')
        
        return response

    async def square_callback(self, goal_handle):
            self.get_logger().info(f'Executing square goal: side_length={goal_handle.request.side_length}')
            feedback_msg = Square.Feedback()
            result = Square.Result()

            side_length = float(goal_handle.request.side_length)
            if ((side_length <= 0) or (side_length > 11.0)):
                result.success = False
                goal_handle.abort()
                return result

            # Properly await the reset logic coroutine
            await self.reset_turtle(Clearc.Request(), Clearc.Response())


            start_x = 5.5
            start_y = 5.5
            counter1 = 0
            corners = [
                (start_x + side_length, start_y, 1.57),
                (start_x + side_length, start_y + side_length, 3.14),
                (start_x, start_y + side_length, 4.71),
                (start_x, start_y, 0.0),
            ]

            for x, y, yaw in corners:
                counter1 += 1
                feedback_msg.side_number = f"Moving to corner {counter1} at ({x}, {y})"
                goal_handle.publish_feedback(feedback_msg)
                move_result = await self.send_go_to_pose(x, y, yaw)
                if move_result is None or not move_result.success:
                    result.success = False
                    goal_handle.abort()
                    return result


            result.success = True
            goal_handle.succeed()
            return result

     

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()