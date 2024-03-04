#!/usr/bin/env python

from __future__ import print_function

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist as TwistMsg
from geometry_msgs.msg import TwistStamped
from vision_msgs.msg import Detection2DArray

import sys
from select import select
from typing import List

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

result_dict = {}
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class YoloPublish(Node):
    def __init__(self,name):
        super().__init__(name)
        self.command_publisher_ = self.create_publisher(TwistMsg,"cmd_vel", 1) 
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rclpy.is_shutdown() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rclpy.sleep(0.5)
            i += 1
            i = i % 5
        if rclpy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rclpy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rclpy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

class CarControl:
    def __init__(self, linear_speed=0.2, angular_speed=0.2):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def move_forward(self):
        twist_msg = TwistMsg()
        twist_msg.linear.x = self.linear_speed
        return twist_msg

    def move_backward(self):
        twist_msg = TwistMsg()
        twist_msg.linear.x = -self.linear_speed
        return twist_msg

    def turn_left(self):
        twist_msg = TwistMsg()
        twist_msg.angular.z = self.angular_speed
        return twist_msg

    def turn_right(self):
        twist_msg = TwistMsg()
        twist_msg.angular.z = -self.angular_speed
        return twist_msg

    def stop(self):
        twist_msg = TwistMsg()
        return twist_msg

    def faster(self):
        self.linear_speed = self.linear_speed+0.1
        return self.linear_speed

    def slower(self):
        self.linear_speed = self.linear_speed-0.1
        return self.linear_speed

class YoloSubscribe(Node):
    def __init__(self,name):
        super().__init__(name)
        self.command_subscribe_ = self.create_subscription(Detection2DArray,"yolo_result",self.command_callback,10)
        self.result_publisher_ = self.create_publisher(TwistMsg,"cmd_vel", 1)
    def command_callback(self,msg):
        result_dict={}
        for detection in msg.detections:
            for result in detection.results:
                result_dict[result.id] = round(result.score,4)
        self.get_logger().info(f'收到[{result_dict}]')
        twist_msg = TwistMsg()
        car=CarControl(0.5,0.1)
        if('bottle' in result_dict.keys()):
            twist_msg=car.turn_left()
        elif('person' in result_dict.keys() and result_dict['person'] < 0.4):
            twist_msg=car.move_forward()
        elif('person' in result_dict.keys()):
            twist_msg=car.stop()
        else:
            twist_msg=car.move_forward()
        self.result_publisher_.publish(twist_msg)
        self.get_logger().info(f'发送[{twist_msg}]')

def main(args=None):
    settings = saveTerminalSettings()

    speed = 0.5
    turn = 1.0
    speed_limit = 1000
    turn_limit = 1000
    repeat = 0.0
    key_timeout = 0.5
    stamped = False
    twist_frame = ''
    if stamped:
        TwistMsg = TwistStamped

    # pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    rclpy.init(args=args)
    node=YoloSubscribe("yolo_control")
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
    