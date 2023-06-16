#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
 

path = Path()

def odom_cb(data):
    #rate = rospy.Rate(5)
    global path
    path.header = data.header
    path.header.frame_id = "map"
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    #rate.sleep( )

rospy.init_node('path_node')

odom_sub = rospy.Subscriber('/camera/odom/sample', Odometry, odom_cb)
path_pub = rospy.Publisher('/path', Path, queue_size=10)

if __name__ == '__main__':
    r = rospy.Rate(1) # 0.5hz
    while not rospy.is_shutdown():
        path_pub.publish(path)
        r.sleep()
        #print("path")
    #rospy.spin()


    
