#!/usr/bin/env python
import rospy
import random
import actionlib
import os
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class DictOp:
    def __init__(self):
        rospy.init_node('dic_op_node', anonymous=True)
        self.cood = dict()
        self.file = rospy.get_param("~data_file","loc.txt")
        self.load_ori_point = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.keep_origin_loc)
        self.add_new_point = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.addTo_goal)

        # Subscriber for clear goals
        self.clear_all_point = rospy.Subscriber("/clear_nav_point",Bool,self.clear_goals)
    
    def read_goals(self):
        """
        read goals from data file
        """
        #set subscriber to get new goal and add it to locations
        if not os.path.exists(self.file):
            rospy.logerr(" %s do not exits",self.file)
            return
        f = open(self.file,'r') 
        for line in f.readlines(): 
            line = line.strip()
            tmp = line.split(' ',1)
            k =tmp[0]
            v = tmp[1]
            v  =  v.strip('[').strip(']').split(', ') 
            for i in range(len(v)):
                v[i]=float(v[i])
            self.cood[k] = [v[0],v[1], v[2],v[3],v[4],v[5],v[6]]
        f.close()

    def clear_goals(self,data):
        """
        clear all record goals
        """
        self.read_goals()
        if(data.data):
            origin = self.cood["1"]
            self.cood = dict()
            self.cood["1"] = origin
            # clear the file
            self.save()
            rospy.loginfo("delete goals!")


    def save(self):
        """
        save the origin and goals to the file
        """
        f = open(self.file,"w")
        for k,v in self.cood.items():
            f.write(str(k)+' '+str(v)+'\n')
        f.close()


    def addTo_goal(self,data):
        if len(self.cood)>0:
            self.cood[str(len(self.cood)+1)]= [data.pose.position.x,data.pose.position.y,data.pose.position.z,\
                                                    data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
            self.save()
        

    def keep_origin_loc(self,data):
        self.cood["1"]=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,\
                                                    data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        self.save()
    
if __name__ == '__main__':
    try:
        DictOp()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Dict exception finished.")