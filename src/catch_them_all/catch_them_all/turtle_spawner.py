#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import random
import math
from interfaces.msg import Turtle
from interfaces.msg import TurtleArray
from interfaces.srv import CatchTurtle

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.declare_parameter('turtle_name_prefix', 'turtle')
        self.turtle_name_prefix = self.get_parameter('turtle_name_prefix').value
        
        self.declare_parameter("spawn_frequency", 1.0)
        self.spawn_frequency = self.get_parameter("spawn_frequency").value

        self.turtle_count = 1
        self.alive_turtles_ = []
        self.alive_turtles_pub = self.create_publisher(TurtleArray, 'alive_turtles', 10)
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.catch_turtle_service = self.create_service(CatchTurtle, 'catch_turtle', self.catch_turtle_callback)
        self.spawn_turtle_timer = self.create_timer(1.0/self.spawn_frequency, self.spawn_new_turtle)

    def catch_turtle_callback(self, request : CatchTurtle.Request, response : CatchTurtle.Response):
        self.call_kill_service(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_pub.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_count += 1
        name = self.turtle_name_prefix + str(self.turtle_count)
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(0.0, 2*math.pi) 
        self.call_spawn_service(x, y, theta, name)

    
    def call_spawn_service(self, x, y, theta, turtle_name):
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for spawn service...')
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request=request))

    def callback_call_spawn_service(self, future, request : Spawn.Request):
        response : Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info(f'Successfully spawned turtle: {response.name}')
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()

    def call_kill_service(self, turtle_name):
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for kill service...')
        
        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, request=request))

    def callback_call_kill_service(self, future, request : Kill.Request):
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.name == request.name:
                del self.alive_turtles_[i]
                self.publish_alive_turtles()
                break


def main(args=None):
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    rclpy.spin(turtle_spawner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()