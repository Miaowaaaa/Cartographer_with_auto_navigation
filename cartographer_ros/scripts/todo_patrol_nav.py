#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Description:
# 在地图上指定6个坐标点作为巡逻点,可以在这些点
# 之间进行不断的巡逻,也可以指定巡逻的圈数,当到
# 达指定的圈数后就会停止运行.该patrol_nav_node,
# 是通过向MoveBaseGoal的target_pose中发布目标
# 位姿来达到巡航的目的,根据move_base的action状态
# 来判断机器人是否到达了目标位置.当到达了目标位置
# 后取出下一个目标位姿进行导航.直到字典存储的所有
# 目标位姿都到达后,就认为巡航一圈结束了.
import os
import sys
import rospy
import random
import actionlib
import time
import re
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion,PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from std_msgs.msg import String

class PatrolNav():

    def __init__(self):
        rospy.init_node('auto_patrol', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # From launch file get parameters
        self.rest_time     = rospy.get_param("~rest_time", 5)
        self.keep_patrol   = rospy.get_param("~keep_patrol",   False)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type   = rospy.get_param("~patrol_type", 0)
        self.patrol_loop   = rospy.get_param("~patrol_loop", 1)
        self.patrol_time   = rospy.get_param("~patrol_time", 5)
        
        self.patrol_file = "/home/Miaow/loc.txt"
        self.labels_file = "/home/Miaow/labels.txt"

         # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']
        
        #flag for stop patrolling
        self.is_stop = False

        #set all navigation target pose
        self.locations = dict()
        self.cood  = dict()
        #set subscriber to get new goal and add it to locations

        # Service for navigation
        self.patrol_service = rospy.Service("/patrol_service",Empty,self.begin_navigation)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # self.move_base.wait_for_server(rospy.Duration(30))
        # rospy.loginfo("Connected to move base server")

        # Subscribe to the voice regnization
        self.voice_reg_sub = rospy.Subscriber("/reg_result",String,self.voice_command)
        
        # read points from data file
        self.locations = self.read_goals(self.patrol_file)
        self.cood = self.read_goals(self.labels_file)
        # init robot pose publisher
        self.init_pose_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=1)
        self.init_pose()
        
    def init_pose(self):
        """
        init robot pose in the exist map
        """
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = rospy.Time().now()
        init_pose.pose.pose = self.locations["1"]
        time.sleep(2)
        self.init_pose_pub.publish(init_pose)
        
    def voice_command(self,data):
        """ 
        voice command
        """
        self.cood = self.read_goals(self.labels_file)
        self.is_stop = True
        regex = re.compile("[a-zA-Z0-9]+")
        label = regex.findall(data.data)
        for l in label:
            rospy.loginfo("parse data is %s.",l)
            if len(l) == 4 and l[0] == '1':
                key = "e" + l[1:]
            else:
                key = l.lower()
            rospy.loginfo("Target is %s.",key)
            if key in self.cood.keys():
                rospy.loginfo(key)
                self.send_goal(self.cood,key)
                finish_time = self.move_base.wait_for_result(rospy.Duration(300))
                if not finish_time:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                    else:
                        rospy.logerr("Goal failed with error code:"+str(self.goal_states[state]))
                else:
                    rospy.logwarn("Time out! can't get the goal")
            else:
                rospy.logwarn("Current map doesn't have the target goal")
        #Go back
        # self.send_goal(self.locations,"1")
        # finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
        # # Check for success or failure
        # if not finished_within_time:
        #     self.move_base.cancel_goal()
        #     rospy.logerr("ERROR:Cannot go back!")
        # else:
        #     state = self.move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         rospy.loginfo("Go Back!")
        #     else:
        #         rospy.logerr("Go Back failed with error code:"+str(self.goal_states[state]))
        # self.is_stop = False
   
    def read_goals(self,filepath):
        dictionary = dict()
        if not os.path.exists(filepath):
            rospy.logerr(filepath)
            return
        f = open(filepath,'r') 
        for line in f.readlines(): 
            line = line.strip()
            tmp = line.split(' ',1)
            k =tmp[0]
            v = tmp[1]
            v  =  v.strip('[').strip(']').split(', ') 
            for i in range(len(v)):
                v[i]=float(v[i])
            dictionary[k] = Pose(Point(v[0],v[1],v[2]), Quaternion(v[3],v[4],v[5],v[6]))
        f.close()
        return dictionary

    def begin_navigation(self,req):
        
        # first, read goals
        self.locations = self.read_goals(self.patrol_file)

        rospy.loginfo("Starting position navigation")
        # Variables to keep track of success rate, running time etc.
        loop_cnt = 0
        n_goals  = 0
        n_successes  = 0
        target_num   = 0
        running_time = 0
        location = ""
        start_time = rospy.Time.now()
        locations_cnt = len(self.locations)
        sequeue = list(self.locations.keys())

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown() and not self.is_stop:
            #judge if set keep_patrol is true
            if self.keep_patrol == False:
                if self.patrol_type == 0: #use patrol_loop
                    if target_num == locations_cnt :   # finish a loop
                        if loop_cnt < self.patrol_loop-1:
                            target_num = 0
                            loop_cnt  += 1
                            rospy.logwarn("Left patrol loop cnt: %d", self.patrol_loop-loop_cnt)
                        else:
                            rospy.logwarn("Now patrol loop over, exit...")
                            break
            else:
                if self.random_patrol == False:
                    if target_num == locations_cnt:
                        target_num = 0
                else:
                    target_num = random.randint(0, locations_cnt - 1)

            # Get the next location in the current sequence
            location = sequeue[target_num]
            rospy.loginfo("Going to: " + str(location))
            self.send_goal(self.locations,location)

            # Increment the counters
            target_num += 1
            n_goals    += 1

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR: Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    n_successes += 1
                    rospy.loginfo("Goal succeeded!")
                else:
                    rospy.logerr("Goal failed with error code:"+str(self.goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs/60.0

            # Print a summary success/failure and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(self.trunc(running_time, 1)) + " min")
            rospy.sleep(self.rest_time)

            if self.keep_patrol == False and self.patrol_type == 1: #use patrol_time
                if running_time >= self.patrol_time:
                    rospy.logwarn("Now reach patrol_time, back to original position...")
                    self.send_goal(self.locations,'six')
                    break
        
        self.send_goal(self.locations,"1")
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.logerr("ERROR: Cannot go back!")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                n_successes += 1
                rospy.loginfo("Go Back!")
            else:
                rospy.logerr("Go Back failed with error code:"+str(self.goal_states[state]))
        return []
        

    def send_goal(self, dictionary,locate):
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = dictionary[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base

    def trunc(self, f, n):
        # Truncates/pads a float f to n decimal places without rounding
        slen = len('%.*f' % (n, f))
        return float(str(f)[:slen])

    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

# def keep_origin_loc(origin): 
#         self.cood["1"]=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,\
#                         data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        
#         f = open("loc.txt",'w')
#         for k,v in self.cood.items():
#             f.write(str(k)+' '+str(v)+'\n')
#         f.close()
#         rospy.loginfo("save successfully,exit")

# def exitfunc():
#     self.reset_loc = rospy.Subscriber('current_pose', ,keep_origin_loc)


if __name__ == '__main__':
    # sys.exitfunc=exitfunc
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")