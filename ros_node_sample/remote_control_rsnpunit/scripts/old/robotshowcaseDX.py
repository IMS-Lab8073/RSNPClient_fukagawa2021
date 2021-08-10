#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
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


import openpyxl
import rospkg
#poseX = [2.6685,2.4511,0.8897,-1.4963,-1.3400]
#poseY = [0.1707,-4.7689,-9.6760,-7.6325,-3.0090]

#orienZ = [0.6368,0.0510,-0.7943,0.9996,-0.9998]
#orienW = [0.7710,0.9986,0.6075,0.02513,0.01585]

#poseX = [4.478,14.212,22.550,-9999999,-99999999]
#poseY = [-0.432,-1.067,-1.349,-99999999,-99999999]

#orienZ = [0.0081,-0.623,-0.672,-0.672,-0.672];
#orienW = [0.999,0.781,0.740,0.740,0.740];

poseX = [0] * 10
poseY = [0] * 10

orienZ = [0] * 10
orienW = [0] * 10

class testNode():
    def __init__(self):
        # MQTT Client
        self.hostname = "192.168.100.118"
        self.mqttc = MQTTClient.MyMQTTClass()
        self.mqttc.run(self.hostname, "fromServer/Velocity")
        
        # ROS
        # Publisher, Subscriber
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("local_area", String, self.roscallback)
        
        # ROS navigator
        self.navigator = GoToPose()

    def roscallback(self, msg):
        # ros callback
        location = msg.data
        
        location_dict = {"A":"1","B":"2","C":"3","D":"4","E":"5"}
        # set odom data
        send_data = {"location_id":location_dict[location]}

        rospy.loginfo("Location : %s , Send data=%s", location, send_data)
        
        send_data_json = json.dumps(send_data)
        # MQTT publish
        self.mqttc.publish_message(self.hostname, "toUnit/Robotdata", send_data_json)

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
            "A":{"position":{"x":poseX[0],"y":poseY[0],"z":0},"quaternion":{"r1":0,"r2":0,"r3":orienZ[0],"r4":orienW[0]}},
            "B":{"position":{"x":poseX[1],"y":poseY[1],"z":0},"quaternion":{"r1":0,"r2":0,"r3":orienZ[1],"r4":orienW[1]}},
            "C":{"position":{"x":poseX[2],"y":poseY[2],"z":0},"quaternion":{"r1":0,"r2":0,"r3":orienZ[2],"r4":orienW[2]}},
            "D":{"position":{"x":poseX[3],"y":poseY[3],"z":0},"quaternion":{"r1":0,"r2":0,"r3":orienZ[3],"r4":orienW[3]}},
            "E":{"position":{"x":poseX[4],"y":poseY[4],"z":0},"quaternion":{"r1":0,"r2":0,"r3":orienZ[4],"r4":orienW[4]}}
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

    rospack = rospkg.RosPack()
    wb = openpyxl.load_workbook(rospack.get_path('odom_listener') + '/xl/Simulation.xlsx')
    sheet = wb.get_sheet_by_name('Sheet1')
    
    count = 2
    while not (sheet['A' + str(count)].value == 'END'):
        poseX[count - 2] = sheet['B' + str(count)].value
        poseY[count - 2] = sheet['C' + str(count)].value
        orienZ[count - 2] = sheet['D' + str(count)].value
        orienW[count - 2] = sheet['E' + str(count)].value
        count += 1
    rospy.init_node('rsnpunitconnector')

    time.sleep(0.5)
    node = testNode()

    while not rospy.is_shutdown():
        # print("run in while")
        if node.mqttc.isNew(): 
            node.mqttcallback()
        rospy.sleep(0.01)
