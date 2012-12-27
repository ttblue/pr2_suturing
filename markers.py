"""
Simple script to display markers in rviz.
Uses John's python code.
"""

import brett2.ros_utils as ru
import rospy
import geometry_msgs.msg as gm


rospy.init_node("markers1")


rviz = ru.RvizWrapper()

ps = gm.PoseStamped()
ps.header.frame_id = '/map'
ps.header.stamp = rospy.Time.now()

# change ps.pose as desired
ps.pose.position.x = 0
ps.pose.position.y = 0
ps.pose.position.z = 0
ps.pose.orientation.x = 0.0
ps.pose.orientation.y = 0.0
ps.pose.orientation.z = 0.0
ps.pose.orientation.w = 1.0

rviz.draw_marker(ps, ns='markers1',id=0,scale=(1,1,1), duration=0)
