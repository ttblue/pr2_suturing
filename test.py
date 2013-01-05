#TODO: Figure out how to pause between actions in trajectory without guessing for times


import numpy as np

import roslib; roslib.load_manifest('filter_cloud_color')
from filter_cloud_color.srv import PointDir, PointDirRequest, PointDirResponse
import rospy
roslib.load_manifest('tf'); import tf
import jds_utils.conversions as conv

import ArmPlannerPR2 
from brett2.PR2 import IKFail

class SutureActions (object):
    
    def __init__(self):
        self.pr2 = ArmPlannerPR2.PlannerPR2()
        
        rospy.wait_for_service('getCutLine')
        self.cutService = rospy.ServiceProxy('getCutLine',PointDir)
        
        rospy.wait_for_service('getHoleNormal')
        self.holeService = rospy.ServiceProxy('getHoleNormal', PointDir)
        
        self.camera_frame = '/camera_rgb_optical_frame'
        # baselinkFromCamera
        self.pr2.tf_listener.waitForTransform("/base_link", self.camera_frame, rospy.Time(), rospy.Duration(10.0))
        (trans, rot) = self.pr2.tf_listener.lookupTransform('/base_link',self.camera_frame,rospy.Time(0))
        self.camera_transform = conv.trans_rot_to_hmat(trans, rot);
        
        # Variables for flap pick-up
        # How far above the cut to start
        self.cut_upStartPos = 0.05
        # How far to the side to start (+ve direction is opposite to the side of the cut)
        # i.e, for left cut, +ve is right and vice versa
        self.cut_sideStartPos = -0.00
        # Rotation of gripper downward  
        self.cut_rotStartPos = np.pi/4
        # Initial gripper angle to hold cut
        self.cut_gripperStartAngle = 0.04
        # How much to move down to pick up flap
        self.cut_moveDown = 0.1
        # How much to move in to pick up flap
        self.cut_moveIn = 0.05
        # How much to move up after picking up the flap
        self.cut_moveUpEnd = 0.03
        # How much to move in after picking up flap 
        self.cut_moveInEnd = 0.015
        
    # Point would need to be midpoint    
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
            print "In camera frame:"
            print "Mid-point of cut: \n", [pt.x, pt.y, pt.z]
            print "Direction of line of cut: \n", [dr.x, dr.y, dr.z]
            return [pt.x,pt.y,pt.z],[dr.x,dr.y,dr.z]
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
            print "In camera frame:"
            print "Average point of hole: \n", [pt.x, pt.y, pt.z]
            print "Direction of normal: \n", [dr.x, dr.y, dr.z]
            return [pt.x,pt.y,pt.z],[dr.x,dr.y,dr.z]
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

        # Flip orientation according to cut        
        flip = {1:1, 2:-1}[index]

        dVecRot = dVecCam + [0]
        
        dVecX = self.camera_transform.dot(dVecRot)[:-1]
        # Rave's base_link not correct way to orient dVecX. 
        # dVecX_baselink = self.pr2.robot.GetLink('base_link').GetTransform()[:-1,0]
        # Orient dVecX to point away from PR2 
        # if dVecX.dot(dVecX_baselink) < 0:
        #     dVecX = -1*dVecX

        dVecY = flip*np.array([-dVecX[1],dVecX[0],0])
        dVecY = dVecY/np.linalg.norm(dVecY)
        dVecZ = np.cross(dVecX, dVecY)
        
        tfm = np.eye(4)
        tfm[0:3,0] = np.unwrap(dVecX)
        tfm[0:3,1] = np.unwrap(dVecY)
        tfm[0:3,2] = np.unwrap(dVecZ)
        tfm[0:3,3] = self.camera_transform.dot(pt+[1])[:-1]
        
        print "Midpoint: ", self.camera_transform.dot(pt+[1])[:-1]
        
        return tfm 
        
    # TODO: Move gripper to some point along cut rather than midPoint
    # TODO: Rotation correction done only for left cut. Do for right cut (left gripper) 
    def moveGripperToPickupStartPos (self, index, dist):
        """
        Moves appropriate gripper to cut specified by index.
        dist is the distance from the midpoint along the cut where
        the PR2 should pick up the flap. +ve direction is away from the PR2.
        """
        arm = {1:self.pr2.rarm, 2:self.pr2.larm}[index]
        # Close gripper before starting
        gripper = {1:self.pr2.rgrip, 2:self.pr2.lgrip}[index]
        gripper.close()
        self.pr2.join_all()
        rospy.sleep(1)

        # Flip orientation of several things according to cut        
        flip = {1:1, 2:-1}[index]
        
        cutTfm = self.findCutTransform(index)
        if cutTfm is None:
            rospy.logerr("Cannot find cut Transform for the index")
            return
        
        """ Prev code:
        cutTfm[2,3] += self.cut_upStartPos
        cutTfm[1,3] -= flip*self.cut_sideStartPos
        """
        
        dVecX = cutTfm[0:3,0]
        dVecY = cutTfm[0:3,1]
        dVecZ = cutTfm[0:3,2]
        cutTfm[0:3,3] += dist*np.unwrap(dVecX) - flip*self.cut_sideStartPos*np.unwrap(dVecY) + flip*self.cut_upStartPos*np.unwrap(dVecZ)
        
        print "New grasp point: ", cutTfm[0:3,3].tolist() 
        
        corrRot = np.array([[-1,0,0,0],
                            [ 0,0,1,0],
                            [ 0,1,0,0],
                            [ 0,0,0,1]])
        # Rotation about z axis by angle = self.cut_rotStartPos
        corrRot2 = np.array ([[1, 0                                , 0                                 , 0],
                              [0, np.cos(flip*self.cut_rotStartPos), -np.sin(flip*self.cut_rotStartPos), 0],
                              [0, np.sin(flip*self.cut_rotStartPos), np.cos(flip*self.cut_rotStartPos) , 0],
                              [0, 0                                , 0                                 , 1]])
        gpTfm = cutTfm.dot(corrRot.dot(corrRot2))
        """ For testing the transforms: ""
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(0.2)
        (trans, rot) = conv.hmat_to_trans_rot(gpTfm)
        
        while True:
            br.sendTransform(trans, rot,
                             rospy.Time.now(),
                             'gripper_tfm',
                             'base_link')
            rate.sleep()
        #"""
        arm.goto_pose_matrix(gpTfm, 'base_link', 'end_effector')
        self.pr2.join_all()
        
    def moveGripperToPickupEndPos (self, index):
        """
        Assumes that the gripper is now holding the flap in the same
        orientation as the start position.
        ONLY call this if the above assumption is true.
        """
        arm = {1:self.pr2.rarm, 2:self.pr2.larm}[index]
        # To move into the cut, in case it is useful
        #moveInDir = {1:'l', 2:'r'}[index]
        flip = {1:1, 2:-1}[index]
        
        gpTfm = arm.manip.GetEndEffectorTransform()
        # Assuming gripper is at an angle
        corrRot = np.array ([[1, 0                                            , 0                                             , 0],
                             [0, np.cos(flip*(np.pi/2 - self.cut_rotStartPos)), -np.sin(flip*(np.pi/2 - self.cut_rotStartPos)), 0],
                             [0, np.sin(flip*(np.pi/2 - self.cut_rotStartPos)), np.cos(flip*(np.pi/2 - self.cut_rotStartPos)) , 0],
                             [0, 0                                            , 0                                             , 1]])
        gpTfm = gpTfm.dot(corrRot)
        
        """ For testing the transforms:
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(0.2)
        (trans, rot) = conv.hmat_to_trans_rot(gpTfm)
        
        while True:
            br.sendTransform(trans, rot,
                             rospy.Time.now(),
                             'gripper_final_tfm',
                             'base_link')
            rate.sleep()
        """
        
        arm.goto_pose_matrix(gpTfm, 'base_link', 'end_effector')
        self.pr2.join_all()
        rospy.sleep(7)
        arm.goInWorldDirection('u',self.cut_moveUpEnd)
        self.pr2.join_all()
        rospy.sleep(7)
        #arm.goInWorldDirection(moveInDir,self.cut_moveInEnd)
        #self.pr2.join_all()
        #rospy.sleep(3)
        
    
    def pickUpFlap (self, index, dist=0.0):
        """
        Function to make PR2 pick up flap of cut represented by index.
        dist -> distance along cut (away from PR2) to pick up cut.
        """
        gripper = {1:self.pr2.rgrip, 2:self.pr2.lgrip}[index]
        arm = {1:self.pr2.rarm, 2:self.pr2.larm}[index]
        moveInDir = {1:'l', 2:'r'}[index]
        
        rospy.loginfo("Going to the start position for picking up the flap.")
        self.moveGripperToPickupStartPos(index, dist)
        rospy.sleep(7)
        gripper.set_angle(self.cut_gripperStartAngle)
        self.pr2.join_all()
        rospy.sleep(2)
        # PR2 is now ready to pick up flap
        # Move down to pick up
        
        # Test code in order to find some solution:
        thresh = 0.001
        attempts = 15
        for count in range(attempts + 1):
            try:
                arm.goInWorldDirection('d', self.cut_moveDown - count*thresh)
                print self.cut_moveDown - count*thresh
                break
            except IKFail:
                if count==attempts:
                    raise IKFail
                else:
                    pass
                
        self.pr2.join_all()
        rospy.sleep(7)
        arm.goInWorldDirection(moveInDir, self.cut_moveIn)
        self.pr2.join_all()
        rospy.sleep(7)
        gripper.close()
        self.pr2.join_all()
        rospy.sleep(4)
        
        #When tested:
        self.moveGripperToPickupEndPos(index)

if __name__=="__main__":
    rospy.init_node("test_node")
    p = SutureActions()
    larm = p.pr2.larm
    rarm = p.pr2.rarm