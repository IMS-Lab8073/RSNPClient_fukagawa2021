#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

import MQTTClient
import time
import json
import math

import openpyxl

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
        self.sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.roscallback)
        
        # ROS navigator
        self.navigator = GoToPose()

    def roscallback(self, msg):
        # ros callback
        location_dict = {"A":15,"B":16,"C":17,"D":18,"E":19}
        x = msg.pose.position.x
        y = msg.pose.position.y
        rospy.loginfo("Point: x=%s y=%s theta=%s", x, y, theta)
        dist = {}
        min_destination = ""
        distination = {"A":[poseX[0],poseY[0]],"B":[poseX[1],poseY[1]],"C":[poseX[2],poseY[2]],"D":[poseX[3],poseY[3]],"E":[poseX[4],poseY[4]]}
        for k, v in destination.items():
            dist[k] = np.linalg.norm(np.array([v[0], v[1]]) - np.array([x, y]))
        min_destination = min(dist, key=dist.get)

        # set robot position data
        send_data = {"location_id":location_dict[min_destination],"data":[{"function_id":7,"data":str(x)+","+str(y)},{"function_id":6,"data":location_dict[min_destination]}]}

        rospy.loginfo("Location : %s , Send data=%s", min_destination, send_data)
        
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
