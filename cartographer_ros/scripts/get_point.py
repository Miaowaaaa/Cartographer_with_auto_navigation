#!/usr/bin/env python
'''
This node is used to record patrol points labelled on rviz map.
'''
import rospy
import random
import actionlib
import os
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class DictOp:
    def __init__(self):
        self.cood = dict()
        self.label_cood = dict()
        self.first = True
        self.patrolfile = "/home/Miaow/loc.txt"
        self.labelsfile = "/home/Miaow/labels.txt"
        self.read_goals(self.cood,self.patrolfile)
        self.read_goals(self.label_cood,self.labelsfile)
        self.label = None
        self.load_ori_point = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.keep_origin_loc)
        self.add_new_point = rospy.Subscriber("/my_nav_goal",PoseStamped,self.addTo_goal)
        self.add_new_label = rospy.Subscriber("/point_label",String,self.add_label)
        # Subscriber for clear goals
        self.clear_all_point = rospy.Subscriber("/clear_nav_point",Bool,self.clear_goals)
        rospy.spin()
    
    def add_label(self,data):
        """
        get a label for point
        """
        self.label = data.data
       
    def read_goals(self,dictionary,file):
        """
        read goals from data file
        """
        #set subscriber to get new goal and add it to locations
        if not os.path.exists(file):
            rospy.logerr(" %s do not exits",file)
            return
        f = open(file,'r') 
        for line in f.readlines(): 
            line = line.strip()
            tmp = line.split(' ',1)
            k = tmp[0]
            v = tmp[1]
            v  =  v.strip('[').strip(']').split(', ') 
            for i in range(len(v)):
                v[i]=float(v[i])
            dictionary[k] = [v[0],v[1], v[2],v[3],v[4],v[5],v[6]]
        f.close()

    def clear_goals(self,data):
        """
        clear all record goals
        """
        self.read_goals(self.cood,self.patrolfile)
        if(data.data == True):
            origin = self.cood["1"]
            self.cood = dict()
            self.cood["1"] = origin
            # clear the file
            self.save(self.cood,self.patrolfile)
            rospy.loginfo("delete goals!")


    def save(self,dictionary,filename):
        """
        save the origin and goals to the file
        """
        
        f = open(filename,"w")
        for k,v in dictionary.items():
            f.write(str(k)+' '+str(v)+'\n')
        f.close()


    def addTo_goal(self,data):
        """
        record a new goal
        """
        if self.label:
            self.label_cood[self.label] = [data.pose.position.x,data.pose.position.y,data.pose.position.z,\
                                                    data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
            self.label = None
            self.save(self.label_cood,self.label_file)
            return
        if len(self.cood)>0:
            self.cood[str(len(self.cood)+1)]= [data.pose.position.x,data.pose.position.y,data.pose.position.z,\
                                                    data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
            self.save(self.cood,self.patrolfile)
        

    def keep_origin_loc(self,data):
        """
        update origin location
        """
        if self.first:        # ignore the initialize by amcl
            self.first = False
            return
        self.cood["1"]=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,\
                                                    data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        self.save(self.cood,self.patrolfile)
    
if __name__ == '__main__':
    rospy.init_node('point_recorder', anonymous=False)
    try:
        DictOp()
    except rospy.ROSInterruptException:
        rospy.logwarn("Dict exception finished.")