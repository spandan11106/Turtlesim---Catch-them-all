#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from interfaces.msg import Turtle
from interfaces.msg import TurtleArray
from interfaces.srv import CatchTurtle

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.declare_parameter('catch_closest_turtle_first', True)
        self.catch_closest_turtle_first = self.get_parameter('catch_closest_turtle_first').value
        self.turtle_to_catch : Turtle = None
        self.pose_ : Pose = None
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.alive_turtles_sub = self.create_subscription(
            TurtleArray,
            'alive_turtles',
            self.alive_turtles_callback,
            10
        )
        self.catch_turtle_client = self.create_client(CatchTurtle, 'catch_turtle')
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)  

    def pose_callback(self, pose: Pose):
        self.pose_ = pose

    def alive_turtles_callback(self, msg: TurtleArray):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

                    if closest_turtle is None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch = closest_turtle
            else:
                self.turtle_to_catch = msg.turtles[0] 

    def control_loop(self):
        if self.pose_ is None or self.turtle_to_catch is None:
            return
        dist_x = self.turtle_to_catch.x - self.pose_.x
        dist_y = self.turtle_to_catch.y - self.pose_.y
        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

        cmd = Twist()
        if distance > 0.5:
            # position
            cmd.linear.x = 2.0 * distance
            # orientation
            angle_to_target = math.atan2(dist_y, dist_x)
            angle_diff = angle_to_target - self.pose_.theta
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            # ensure the angular velocity is proportional to the angle difference
            cmd.angular.z = 6.0 * angle_diff
            
        else:
            # target reached, stop the turtle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None  # Reset the target turtle after catching

        self.velocity_publisher.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for catch turtle service...')
        
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, request=request))

    def callback_call_catch_turtle_service(self, future, request: CatchTurtle.Request):
        response: CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error(f'Failed to catch turtle: {request.name}')
        

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()