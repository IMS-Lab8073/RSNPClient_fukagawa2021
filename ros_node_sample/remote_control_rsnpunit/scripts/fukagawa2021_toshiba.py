#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

from geometry_msgs.msg import Twist

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

if __name__ == '__main__':
    rospy.init_node('rsnpunitconnector')

    time.sleep(0.5)
    node = testNode()

    while not rospy.is_shutdown():
        # print("run in while")
        if node.mqttc.isNew(): 
            node.mqttcallback()
        rospy.sleep(0.01)
