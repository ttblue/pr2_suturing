import numpy as np

import roslib; roslib.load_manifest('filter_cloud_color')
from filter_cloud_color.srv import PointDir, PointDirRequest, PointDirResponse
import rospy
roslib.load_manifest('tf'); import tf
import jds_utils.conversions as conv

import ArmPlannerPR2 


class SutureActions (object):
    
    def __init__(self):
        self.pr2 = ArmPlannerPR2.PlannerPR2()
        
        rospy.wait_for_service('getCutLine')
        self.cutService = rospy.ServiceProxy('getCutLine',PointDir)
        
        rospy.wait_for_service('getHoleNormal')
        self.holeService = rospy.ServiceProxy('getHoleNormal', PointDir)
        
        self.camera_frame = '/camera_rgb_optical_frame'
        #baselinkFromCamera
        (trans, rot) = self.pr2.tf_listener.lookupTransform('/base_link',self.camera_frame,rospy.Time(0))
        self.camera_transform = conv.trans_rot_to_hmat(trans, rot);
    
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
            print pt, '\n', dr
            return [pt.x,pt.y,pt.z],[dr.z,dr.y,dr.z]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None, None

    
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
            print pt, '\n', dr
            return [pt.x,pt.y,pt.z],[dr.z,dr.y,dr.z]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None, None

    def findCutTransform (self, index):
        """
        Finds the cut transform at the midpoint of the visible cut.
        X+ Axis is taken to be along the cut moving away from the PR2. 
        Assuming index=1 is the left cut and index=2 is the right cut.
        """
        pt, dVecCam = self.getCutLine(index)
        if pt is None or dVecCam is None: return None
        
        if index==1:
            flip=1
        elif index==2:
            flip=-1
        
        dVecRot = dVecCam + [0]
        
        dVecX = self.camera_transform.dot(dVecRot)[:-1]
        dVecX_baselink = self.pr2.robot.GetLink('base_link').GetTransform()[:-1,0]
        #Orient dVecX to point away from PR2
        if dVecX.dot(dVecX_baselink) < 0:
            dVecX = -1*dVecX

        dVecY = flip*np.array([-dVecX[1],dVecX[0],0])
        dVecY = dVecY/np.linalg.norm(dVecY)
        dVecZ = np.cross(dVecX, dVecY)
        
        tfm = np.eye(4)
        tfm[0:3,0] = np.unwrap(dVecX)
        tfm[0:3,1] = np.unwrap(dVecY)
        tfm[0:3,2] = np.unwrap(dVecZ)
        tfm[0:3,3] = self.camera_transform.dot(pt+[1])[:-1]
        
        print tfm
        
        return tfm 
        
    def moveGripperToCut (self, index):
        """
        Moves appropriate gripper to cut specified by index.
        """
        arm = {1:self.pr2.rarm, 2:self.pr2.larm}[index]
        
        cutTfm = self.findCutTransform(index)
        if cutTfm is None:
            print "Cannot find cut Transform for the index"
            return
        
        corrRot = np.array([[-1,0,0,0],
                            [ 0,0,1,0],
                            [ 0,1,0,0],
                            [ 0,0,0,1]])
        gpTfm = corrRot.dot(cutTfm)
    
        arm.goto_pose_matrix(gpTfm, 'base_link', 'end_effector')

if __name__=="__main__":
    rospy.init_node("test_node")
    p = SutureActions()