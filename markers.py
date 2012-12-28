"""
Simple script to display markers in rviz.
Uses John's python code.
"""

import rospy
import geometry_msgs.msg as gm
import std_msgs.msg as stdm
import numpy as np
from visualization_msgs.msg import Marker


class MarkerPlacer:
    """
    A very simple class to place markers in rviz.
    Useful for visualizing transforms.
    """

    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker)
        self.ids = set([])

    def draw_marker(self, ps, id=None, type=Marker.CUBE, ns='default_ns',rgba=(0,1,1,0.5), scale=(.1,.03,.03),text='',duration=0):
        if id is None: id = self.get_unused_id()
        marker        = Marker(type=type, action=Marker.ADD)
        marker.ns     = ns
        marker.header = ps.header
        marker.pose   = ps.pose
        marker.scale  = gm.Vector3(*scale)
        marker.color  = stdm.ColorRGBA(*rgba)
        marker.id     = id
        marker.text   = text
        marker.lifetime = rospy.Duration(duration)
        self.pub.publish(marker)      

    def get_unused_id(self):
        while True:
            id = np.random.randint(0,2147483647)
            if id not in self.ids: return id
    

rospy.init_node("markers")
r = rospy.Rate(1)
p = MarkerPlacer()

while True:
    ps = gm.PoseStamped()
    ps.header.frame_id = '/frame'
    ps.header.stamp = rospy.Time.now()

    # change ps.pose as desired
    ps.pose.position.x = 0
    ps.pose.position.y = 0
    ps.pose.position.z = 0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = 0.0
    ps.pose.orientation.w = 1.0

    p.draw_marker(ps,scale=(1,1,1), duration=0)
    r.sleep()