#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry   
import tf
from tf.transformations import euler_from_quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import MQTTClient
import time
import json
import math

class testNode():
    def __init__(self):
        # MQTT Client
        self.hostname = "localhost"
        self.mqttc = MQTTClient.MyMQTTClass()
        self.mqttc.run(self.hostname, "fromServer/Velocity")
        
        # ROS
        # Publisher, Subscriber
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("odom", Odometry, self.roscallback)
        
        # ROS navigator
        self.navigator = GoToPose()

    def roscallback(self, msg):
        # ros callback
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        q = (qx, qy, qz, qw)
        e = euler_from_quaternion(q)
        odom_theta = e[2] 
        # rospy.loginfo("Odomery: x=%s y=%s theta=%s", odom_x, odom_y, odom_theta)
        # set odom data
        send_data = {"data_type":"odometry", "data":""}
        send_data["data"] = "x:"+str(odom_x)+",y:"+str(odom_y)+",heading"+str(odom_theta)
        send_data_json = json.dumps(send_data)
        # MQTT publish
        self.mqttc.publish_message(self.hostname, "toUnit/RobotData", send_data_json)

    def mqttcallback(self):
        # mqtt callback
        recievedata =  self.mqttc.recieve_data
        recievedata = json.loads(recievedata)
        cmd_vel = Twist()
        cmd_vel.linear.x = float(recievedata["vx"])  / 1000
        cmd_vel.angular.z = float(recievedata["va"]) / 1000
        rospy.loginfo("Data from RSNP: vx=%s va=%s option=%s", cmd_vel.linear.x, cmd_vel.angular.z, recievedata["option"])
        self.Publisher(cmd_vel)
        if recievedata["option"]!="": self.gotopose(recievedata["option"])

    def gotopose(self, position_name):
        degree = 0
        goal_point = {
            "A":{"position":{"x":1.22,"y":-0.138,"z":0},"quaternion":{"r1":0,"r2":0,"r3":math.sin(math.radians(degree/2)),"r4":math.cos(math.radians(degree/2))}},
            "B":{"position":{"x":4,"y":-1,"z":0},"quaternion":{"r1":0,"r2":0,"r3":-1,"r4":0}},
            "C":{"position":{"x":1.433,"y":-0.9,"z":0},"quaternion":{"r1":0,"r2":0,"r3":-1,"r4":0}},
            "D":{"position":{"x":1,"y":0,"z":0},"quaternion":{"r1":0,"r2":0,"r3":math.sin(math.radians(degree/2)),"r4":math.cos(math.radians(degree/2))}},
            "E":{"position":{"x":0,"y":1,"z":0},"quaternion":{"r1":0,"r2":0,"r3":math.sin(math.radians(degree/2)),"r4":math.cos(math.radians(degree/2))}}
        }

        rospy.loginfo("Go to (%s, %s) pose", goal_point[position_name]["position"]["x"], goal_point[position_name]["position"]["y"])

        rospy.loginfo(goal_point[position_name])

        position   = goal_point[position_name]["position"]
        quaternion = goal_point[position_name]["quaternion"]
        success = self.navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Reached to desired position & pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

    def Publisher(self, data):
        self.pub.publish(data)


class GoToPose():
    def __init__(self):
        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        # Send a goal
        rospy.loginfo(pos,quat)
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                    Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

    	# Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state   = self.move_base.get_state()
        result  = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent: self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('rsnpunitconnector')

    time.sleep(0.5)
    node = testNode()

    while not rospy.is_shutdown():
        # print("run in while")
        if node.mqttc.isNew(): 
            node.mqttcallback()
        rospy.sleep(0.01)