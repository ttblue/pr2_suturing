import rospy

import roslib; roslib.load_manifest('filter_cloud_color')
from filter_cloud_color.srv import PointDir, PointDirRequest, PointDirResponse

import ArmPlannerPR2 


class SutureActions (object):
    
    def __init__(self):
        self.pr2 = ArmPlannerPR2.PlannerPR2()
        
        rospy.wait_for_service('getCutLine')
        self.cutService = rospy.ServiceProxy('getCutLine',PointDir)
        
        rospy.wait_for_service('getHoleNormal')
        self.holeService = rospy.ServiceProxy('getHoleNormal', PointDir)
    
    #Point would need to be midpoint    
    def getCutLine(self, index):
        """
        Calls service 'getCutLine' with index and
        returns a point on the cut and the direction
        of the line through it.
        """
        try:
            response = self.cutService(index)
            pt = response.point
            dr = response.dir
            return [pt.x,pt.y,pt.z],[dr.z,dr.y,dr.z]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    
    def getHoleNormal(self, index):
        """
        Calls service 'getHoleNormal' with index and
        returns the midpoint of the hole and the direction
        of the normal at the hole.
        """
        try:
            response = self.holeService(index)
            pt = response.point
            dr = response.dir
            return [pt.x,pt.y,pt.z],[dr.z,dr.y,dr.z]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def findGripperTransform (self, index):
         """
         Finds the final transform of the gripper needed
         to hold the flap of the cut. This is not the 
         transform from which you start moving towards the 
         cut.
         """
         

if __name__=="__main__":
    rospy.init_node("test_node")
    p = 