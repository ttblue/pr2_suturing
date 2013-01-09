#TODO: Figure out how to pause between actions in trajectory without guessing for times
import numpy as np
import numpy.linalg as nla

import roslib; roslib.load_manifest('filter_cloud_color')
from filter_cloud_color.srv import PointDir, PointDirRequest, PointDirResponse
import rospy
roslib.load_manifest('tf'); import tf
import jds_utils.conversions as conv

from ArmPlannerPR2 import PlannerPR2 
from brett2.PR2 import IKFail

class SutureActionsPR2 (PlannerPR2):
    
    def __init__(self):
        PlannerPR2.__init__(self)
        
        rospy.wait_for_service('getCutLine')
        self.cutService = rospy.ServiceProxy('getCutLine',PointDir)
        
        rospy.wait_for_service('getHoleNormal')
        self.holeService = rospy.ServiceProxy('getHoleNormal', PointDir)
        
        self.camera_frame = '/camera_rgb_optical_frame'
        # basefootprintFromCamera, assuming transform doesn't change. If it does, need to do this often.
        self.tf_listener.waitForTransform("/base_footprint", self.camera_frame, rospy.Time(), rospy.Duration(10.0))
        (trans, rot) = self.tf_listener.lookupTransform('/base_footprint',self.camera_frame,rospy.Time(0))
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
        self.cut_moveUpEnd = 0.09
        # How much to move in after picking up flap 
        self.cut_moveInEnd = 0.015
        
        # Distance to move towards hole from start position to pierce
        self.hole_moveToward = 0.025
        # Radius to move in circle to pierce cut
        self.hole_finAng = np.pi/3 
        
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
        # Rave's base_footprint not correct way to orient dVecX. 
        # dVecX_basefp = self.robot.GetLink('base_footprint').GetTransform()[:-1,0]
        # Orient dVecX to point away from PR2 
        # if dVecX.dot(dVecX_basefp) < 0:
        #     dVecX = -1*dVecX

        dVecY = flip*np.array([-dVecX[1],dVecX[0],0])
        dVecY = dVecY/np.linalg.norm(dVecY)
        dVecZ = np.cross(dVecX, dVecY)
        
        tfm = np.eye(4)
        tfm[0:3,0] = np.unwrap(dVecX)
        tfm[0:3,1] = np.unwrap(dVecY)
        tfm[0:3,2] = np.unwrap(dVecZ)
        tfm[0:3,3] = self.camera_transform.dot(pt+[1])[:-1]
        
        # self.testTransforms(tfm, 'cut_midpoint_tfm', 'camera_rgb_optical_frame')
    
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
        cutTfm = self.findCutTransform(index)
        if cutTfm is None:
            rospy.logerr("Cannot find cut Transform for the index")
            return False
        
        self.update_rave()
        arm = {1:self.rarm, 2:self.larm}[index]
        
        # Close gripper before starting
        gripper = {1:self.rgrip, 2:self.lgrip}[index]
        gripper.close()
        self.join_all()
        rospy.sleep(1)

        # Flip orientation of several things according to cut        
        flip = {1:1, 2:-1}[index]
        
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
        
        # For testing the transforms:
        # self.testTransforms (gpTfm, 'gripper_tfm', 'base_footprint')
        
        arm.goto_pose_matrix(gpTfm, 'base_footprint', 'end_effector')
        self.join_all()
        return True
        
    def moveGripperToPickupEndPos (self, index):
        """
        Assumes that the gripper is now holding the flap in the same
        orientation as the start position.
        ONLY call this if the above assumption is true.
        """
        self.update_rave()
        arm = {1:self.rarm, 2:self.larm}[index]
        # To move into the cut, in case it is useful
        # moveInDir = {1:'l', 2:'r'}[index]
    
        # Removed the following to make more space for the needle
        # flip = {1:1, 2:-1}[index]
        # gpTfm = arm.manip.GetEndEffectorTransform() 
        # Assuming gripper is at an angle, twist it all the way to pi (or -pi)
        # corrRot = np.array ([[1, 0                                            , 0                                             , 0],
        #                      [0, np.cos(flip*(np.pi/2 - self.cut_rotStartPos)), -np.sin(flip*(np.pi/2 - self.cut_rotStartPos)), 0],
        #                      [0, np.sin(flip*(np.pi/2 - self.cut_rotStartPos)), np.cos(flip*(np.pi/2 - self.cut_rotStartPos)) , 0],
        #                      [0, 0                                            , 0                                             , 1]])
        # gpTfm = gpTfm.dot(corrRot)
        
        # For testing the transforms:
        # self.testTransforms (gpTfm, 'gripper_end_tfm', 'base_footprint')
        
        # arm.goto_pose_matrix(gpTfm, 'base_footprint', 'end_effector')
        # self.join_all()
        # rospy.sleep(7)
        
        # Pull up the flap 
        arm.goInWorldDirection('u',self.cut_moveUpEnd)
        self.join_all()
        rospy.sleep(7)
        
        # In case 
        # arm.goInWorldDirection(moveInDir,self.cut_moveInEnd)
        # self.join_all()
        # rospy.sleep(3)
        
    
    def pickUpFlap (self, index, dist=0.0):
        """
        Function to make PR2 pick up flap of cut represented by index.
        dist -> distance along cut (away from PR2) to pick up cut.
        """
        gripper = {1:self.rgrip, 2:self.lgrip}[index]
        arm = {1:self.rarm, 2:self.larm}[index]
        moveInDir = {1:'l', 2:'r'}[index]
        
        rospy.loginfo("Going to the start position for picking up the flap.")
        
        try:
            ready = self.moveGripperToPickupStartPos(index, dist)
            if not ready:
                rospy.logerr("Unable to move to start position to pick up flap.")
                return
        except IKFail:
            rospy.logerr("Unable to move to start position to pick up flap. IK Failed.")
            return
            
        rospy.sleep(7)
        gripper.set_angle(self.cut_gripperStartAngle)
        self.join_all()
        rospy.sleep(2)
        # PR2 is now ready to pick up flap
        # Move down to pick up
        
        # Test code in order to find some solution:
        self.update_rave()
        thresh = 0.001
        attempts = 15 
        for count in range(attempts + 1):
            try:
                arm.goInWorldDirection('d', self.cut_moveDown - count*thresh)
                print "Found distance to move down: ", self.cut_moveDown - count*thresh
                break
            except IKFail:
                if count==attempts:
                    rospy.logerr("Unable to move down. IK Failed.")
                    return
                else:
                    pass
                
        try:
            self.join_all()
            rospy.sleep(7)
            arm.goInWorldDirection(moveInDir, self.cut_moveIn)
            self.join_all()
            rospy.sleep(7)
            gripper.close()
            self.join_all()
            rospy.sleep(4)
        except IKFail:
            rospy.logerr("Unable to pick up flap. IK Failed.")
            return
        #When tested:
        try:
            self.moveGripperToPickupEndPos(index)
        except IKFail:
            rospy.logerr("Unable to move to final position. IK Failed.")
        
    def findHoleTransform (self, index):
        """
        Finds the transform of the needle tip in order to pierce the hole 
        specified by index.
        """
        pt, dVecCam = self.getHoleNormal(index)
        if pt is None or dVecCam is None: return None
        
        # Flip orientation according to hole        
        flip = {1:1, 2:-1}[index]

        
        dVecRot = dVecCam + [0]
        # dVecZ points in the opposite direction of the normal
        dVecZ = -1*self.camera_transform.dot(dVecRot)[:-1]
        
        # Since all transforms are w.r.t base_footprint, its transform is just the identity
        dVecX_bfp = np.array([1,0,0])
        dVecXz_bfp = (dVecX_bfp.dot(dVecZ)/nla.norm(dVecZ, 2))*dVecZ
        dVecXx_bfp = dVecX_bfp - dVecXz_bfp
        # dVecX points in the closest direction to the x axis of the base_footprint frame (identity) 
        dVecX = flip*dVecXx_bfp/nla.norm(dVecXx_bfp,2)
        
        dVecY = np.cross(dVecZ, dVecX)
        
        WfmNTip = np.eye(4)
        WfmNTip[0:3,0] = np.unwrap(dVecX)
        WfmNTip[0:3,1] = np.unwrap(dVecY)
        WfmNTip[0:3,2] = np.unwrap(dVecZ)
        WfmNTip[0:3,3] = self.camera_transform.dot(pt+[1])[:-1]

        # For testing the transforms:
        # self.testTransforms (WfmNTip, 'needle_tip_tfm', 'base_footprint')

        return WfmNTip
    
    # TODO: Maybe start a little way away from the hole to begin with
    # Might not use this if planning entire path is needed beforehand
    def pierceHole(self, index):
        """
        # Moves the gripper to the starting position to pierce the 
        # hole specified by the index.
        # This is assuming the needle is already being held by the
        # gripper.
        
        Function to make the PR2 pierce the hole specified by index.
        If pi/4 works, stick with it (maybe not)
        """ 
        self.update_rave()
        holeTfm = self.findHoleTransform(index)
        if holeTfm is None:
            rospy.logerr("Cannot find hole Transform for the index")
            return False
        
        arm = {1:self.larm, 2:self.rarm}[index]
        flip = {1:1, 2:-1}[index]
        
        # Openrave needle
        self.grabNeedle()
        
        EEfmNTip = self.getGripperFromNeedleTipTransform()
        if EEfmNTip is None:
            rospy.logwarn("Unable to find transform between gripper and needle.")
            return False
        
        rotX = np.eye(4)
        ind1, ind2 = [1,1,2,2], [1,2,1,2]
        theta = -np.pi/4
        rotX[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])        
        
        gpTfm = holeTfm.dot(rotX).dot(nla.inv(EEfmNTip))
        
        gpTfm[0:3,3] += self.hole_moveToward*holeTfm.dot(rotX)[0:3,2]
        
        # For testing the transforms:
        # self.testTransforms (gpTfm, 'new_needle', 'base_footprint')
        
        arm.goto_pose_matrix (gpTfm, 'base_footprint', 'end_effector')
        self.join_all()
        rospy.sleep(10)
        
        arm.circleAroundRadius (self.sneedle_pose, self.sneedle_radius, self.hole_finAng)
        self.join_all()
        rospy.sleep(7)
        
        return True
    
    # Decided to do everything in this function in order to be able to plan entire
    # path and see if there is an IKFail
    def pierceHole2 (self, index):
        """
        Function to make the PR2 pierce the hole specified by index.
        This function checks a bunch of viable positions and chooses the
        one which has the needle closest to the normal.
        """
        arm = {1:self.larm, 2:self.rarm}[index]
        flip = {1:1, 2:-1}[index]
        self.update_rave()
        
        holeTfm = self.findHoleTransform(index)
        if holeTfm is None:
            rospy.logerr("Cannot find hole Transform for the index")
            return
        
        # Openrave needle
        self.grabNeedle()
        
        EEfmNTip = self.getGripperFromNeedleTipTransform()
        if EEfmNTip is None:
            rospy.logwarn("Unable to find transform between gripper and needle.")
            return
        
        rospy.loginfo("Going to the start position for picking up the flap.")

        # Setting up things for planning 
        # Angle to move in circle to pierce, from pi/2 to pi/4
        attempts = 15 
        dAng = np.pi/(4*attempts)
        # If using right arm
        if index == 2:
            pose = {1:2, 2:1, 3:4, 4:3}[self.sneedle_pose]
        else:
            pose = self.sneedle_pose
        d,t = {1:(-1,-1), 2:(1,1), 3:(-1,1), 4:(1,-1)}[pose]
        
        for count in range(attempts + 1):
            try:
                # Need to update rave every time. Actually don't but might as well.
                self.update_rave()
                
                rotX = np.eye(4)
                ind1, ind2 = [1,1,2,2], [1,2,1,2]
                theta = -dAng*count
                rotX[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])        
                gpTfm = holeTfm.dot(rotX).dot(nla.inv(EEfmNTip))
                # gpTfm[0:3,3] += self.hole_moveToward*holeTfm.dot(rotX)[0:3,2]        
                
                arm.goto_pose_matrix_rave(gpTfm, 'base_footprint', 'end_effector')
                
                circleTraj = arm.planner.circleAroundRadius (d, t, self.sneedle_radius, self.hole_finAng)
                if not circleTraj:
                    raise IKFail
                
                # Potentially make count more to be safer
                print "Final angle of needle (0 being normal to hole): ", dAng*count
                break
            except IKFail:
                if count==attempts:
                    rospy.logerr("Unable to pierce flap. IK Failed.")
                    return
                else:
                    pass
        
        # self.testTransforms(gpTfm, 'pierce_frame', 'base_footprint')
        
        # Now that PR2 is ready, ask it to pierce the flap 
        # (and the planner has returned something feasible)
        self.update_rave()
        # Last saved value of gpTfm
        arm.goto_pose_matrix (gpTfm, 'base_footprint', 'end_effector')
        self.join_all()
        rospy.sleep(12)
        # Should work since there was no IKFail
        arm.circleAroundRadius (self.sneedle_pose, self.sneedle_radius, self.hole_finAng)
        self.join_all()
        rospy.sleep(7)
        
    def runThrough (self, index, dist=0.05):
        """
        Picks up indexed flap and pierces indexed cut.
        Distance refers to the distance from the hole to pick up the flap.
        """
        self.pickUpFlap(index, dist)
        self.pierceHole(index)
        
        
    def testNeedleTfm (self):
        """
        Tests Needle transform.
        """
        self.update_rave()
        sTfm = self.needleTipTransform()
        self.testTransforms (sTfm, 'needle_tip', 'base_footprint')
        
    def testTransforms (self, tfm, child_frame, parent_frame):
        """
        Basic code to test transforms. Visualize on RViz or something.
        """
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(0.2)
        (trans, rot) = conv.hmat_to_trans_rot(tfm)
        
        while True:
            br.sendTransform(trans, rot,
                             rospy.Time.now(),
                             child_frame,
                             parent_frame)
            rate.sleep()
        

def waitAndClose(t):
    """
    Waits for t seconds and closes PR2's left gripper.
    """
    rospy.sleep(t)
    p.lgrip.close(80)

if __name__=="__main__":
    rospy.init_node("test_node")
    p = SutureActionsPR2()
    larm = p.larm
    rarm = p.rarm