from IKPlannerFunctions import IKInterpolationPlanner
from brett2.PR2 import PR2, Arm, IKFail

import rospy

import openravepy as opr
import numpy as np
import numpy.linalg as nla

class PlannerArm(Arm):
    """
    Planner class for the Arm.
    """
    def __init__ (self, pr2, rl):
        Arm.__init__(self, pr2, rl)
        self.planner = IKInterpolationPlanner(self, self.lr)

    def goInDirection (self, d, dist, steps=10):
        """
        Moves the tool tip in the specified direction in the gripper frame.
         
        Direction of movement                    -> d
            f -> forward (along the tip of the gripper)
            b -> backward
            u -> up
            d -> down
            l -> left
            r -> right
        Distance traveled by tool tip            -> dist
        Number of points of linear interpolation -> steps
        """
        self.pr2.update_rave()
        trajectory = self.planner.goInDirection(d, dist,steps)
        
        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail


    def goInWorldDirection (self, d, dist, steps=10):
        """
        Moves the tool tip in the specified direction in the base_footprint frame. 
        
        Direction of movement                    -> d
            f -> forward
            b -> backward
            u -> up
            d -> down
            l -> left
            r -> right
        Distance traveled by tool tip            -> dist
        Number of points of linear interpolation -> steps
        """
        self.pr2.update_rave()
        trajectory = self.planner.goInWorldDirection(d, dist,steps)
        
        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail
        

    def circleAroundRadius (self, pose, rad, finAng, steps=10):
        """
        Moves the gripper in a circle.
        
        Pose of needle in hand (as given below in the
          function ArmPlannerPR2.moveNeedleToGripperPose) -> pose
        Radius of circle                                  -> rad
        Final angle covered by circle                     -> finAng
        Number of points of linear interpolation of angle -> steps
        """        
        self.pr2.update_rave()
        
        if pose not in [1,2,3,4]:
            rospy.logwarn ("Invalid option for pose.")
            return
        if self.lr == 'r':
            pose = {1:2, 2:1, 3:4, 4:3}[pose]
        d,t = {1:(-1,-1), 2:(1,1), 3:(-1,1), 4:(1,-1)}[pose]
        
        trajectory = self.planner.circleAroundRadius (d, t, rad, finAng, steps)

        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail
        
    def goto_pose_matrix (self, matrix4, ref_frame, targ_frame, filter_options=-1):
        """
        Moves the arm such that the transform between ref_frame and
        targ_frame is matrix4. Also uses filter_options as specified
        in self.planner (by default).
        """
        if filter_options==-1:
            filter_options = self.planner.filter_options
        Arm.goto_pose_matrix(self, matrix4, ref_frame, targ_frame, filter_options)
        
    def goto_pose_matrix_rave (self, matrix4, ref_frame, targ_frame, filter_options=-1):
        """
        Moves the arm in openrave only. This is only for planning. Please be very careful when
        doing this and make sure to update rave properly back to the real PR2 after finishing.
        """
        if filter_options==-1:
            filter_options = self.planner.filter_options
        joints = self.cart_to_joint(matrix4, ref_frame, targ_frame, filter_options)
        if joints is not None: self.pr2.robot.SetJointValues(joints, self.manip.GetArmIndices())
        else: raise IKFail
    
    def cart_to_joint(self, matrix4, ref_frame, targ_frame, filter_options=-1):
        """
        Calculates IK for given transform between ref_frame and targ_frame. 
        """
        if filter_options==-1:
            filter_options = self.planner.filter_options
        return Arm.cart_to_joint(self, matrix4, ref_frame, targ_frame, filter_options)
        
        

class PlannerPR2 (PR2):
    """
    Planner class for PR2 with planning arms.
    """    
    def __init__ (self,initPos=None):
        PR2.__init__(self)
        self.rarm = PlannerArm(self,'r')
        self.larm = PlannerArm(self,'l')
        
        # In case needle is not added
        self.sneedle = None
        self.sneedle_radius = 0.112
        self.sneedle_pose = 0
        self.grabbingArm = None
        
        # In case sponge is not added (supporting sponge)
        self.sponge = None
        self.sponge_enabled = False
        
        if initPos is not None:
            try:
                self.gotoArmPosture(initPos)
            except:
                rospy.logwarn ("Cannot go to pose " + str(initPos))
                
        self.addTableToRave()
        self.addSpongeToRave()
        self.addNeedleToRave()
        
    def gotoArmPosture (self, pos):
        """
        Makes both arms go to the specified posture.
        """
        self.larm.goto_posture(pos)
        self.rarm.goto_posture(pos)
        self.join_all()
        
    def addTableToRave (self):
        """
        Adds a box of predefined position/ half-extents to
        the rave Environment.
        """
        tablePos = [0.75,0,0.72]
        tableHalfExtents = [0.5,0.45,0.05]
        
        with self.env:
            body = opr.RaveCreateKinBody(self.env,'')
            body.SetName('table')
            body.InitFromBoxes(np.array([tablePos + tableHalfExtents]),True)
            self.env.AddKinBody(body,True)
            
    def addSpongeToRave (self):
        """
        Adds a sponge to account for the supporting sponge of the foam block.
        """
        spongePos = [0.5,0,0.8]
        spongeHalfExtents = [0.25,0.3,0.03]
        
        with self.env:
            self.sponge = opr.RaveCreateKinBody(self.env,'')
            self.sponge.SetName('table')
            self.sponge.InitFromBoxes(np.array([spongePos + spongeHalfExtents]),True)
            self.env.AddKinBody(self.sponge,True)
        
        self.enableSponge(False)    
        
    def enableSponge (self, enable):
        """
        enable -> True/False
        Enables or disables sponge in the environment.
        Consider when motion planning needs to account for sponge collision.
        """
        if self.sponge is None:
            self.sponge_enabled = False
            return
        if enable:
            self.sponge_enabled = True
            self.sponge.Enable(True)
            rospy.loginfo('Supporting sponge enabled for collision checking.')
        else:
            self.sponge_enabled = False
            self.sponge.Enable(False)
            rospy.loginfo('Supporting sponge disabled for collision checking.')

    def addNeedleToRave (self):
        """
        Adds the suturing needle from the .dae file.
        Needle is kept at the edge of the table to start with.
        """
        with self.env:
            self.sneedle = self.env.ReadKinBodyURI('/home/ankush/sandbox/bulletsim/data/needle/sneedle.dae')
            self.sneedle.SetName('sneedle')
            self.env.AddKinBody(self.sneedle)
            
        self.resetNeedlePose()
            
    def resetNeedlePose (self):
        """
        Reset needle transform to be at the edge of the table
        """
        if self.sneedle is None:
            return
        
        sndTfm = np.eye(4)
        sndTfm[0:3,3] = np.array([1.1,-0.11,0.78])
        self.sneedle.SetTransform(sndTfm)
        
    def moveNeedleToGripperPose (self, rl='l', pose=1):
        """
        Resets needle to different poses. All explanations below assume that
        the arm is in the "side" position.
        Positions for right arm are opposite (in terms of left and right) to
        the following.
        Pose 1 - needle is in front of the gripper and pointing away.
        Pose 2 - needle is in front of the gripper and pointing inward.
        Pose 3 - needle is behind the gripper and pointing away.
        Pose 4 - needle is behind the gripper and pointing inward.
        Pose 1 <=> Pose 4 and Pose 2 <=> Pose 3.
        """
        if self.sneedle is None:
            return
        
        if rl == 'r' and pose in [1,2,3,4]:
            pose = {1:2, 2:1, 3:4, 4:3}[pose]
            
        arm = {'l':self.larm, 'r':self.rarm}[rl]
        WfromEE = arm.manip.GetTransform()
        
        trans = np.eye(4)
        trans[[0,1],[3,3]] = np.array([-0.07,-0.05])

            
        if pose == 1:
            rot = np.eye(4)
            ind1, ind2 = [0,0,1,1], [0,1,0,1]
            theta = np.pi/6
            rot[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
        
        elif pose == 2:
            rot1 = np.eye(4)
            ind1, ind2 = [1,1,2,2], [1,2,1,2]
            theta = np.pi
            rot1[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
            
            rot2 = np.eye(4)
            ind1, ind2 = [0,0,1,1], [0,1,0,1]
            theta = np.pi/6
            rot2[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])

            rot = rot1.dot(rot2)
            
        elif pose == 3:
            rot1 = np.eye(4)
            ind1, ind2 = [0,0,2,2], [0,2,0,2]
            theta = np.pi
            rot1[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
            
            rot2 = np.eye(4)
            ind1, ind2 = [0,0,1,1], [0,1,0,1]
            theta = np.pi/6
            rot2[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])

            rot = rot1.dot(rot2)
                
        elif pose == 4:
            rot = np.eye(4)
            ind1, ind2 = [0,0,1,1], [0,1,0,1]
            theta = np.pi/6 + np.pi
            rot[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
            
        else:
            rospy.logwarn ("Unknown needle pose. Not setting any transform.")
            return
                                        
        sndTfm = WfromEE.dot(rot.dot(trans))
        self.sneedle.SetTransform(sndTfm)
        
    def grabNeedle (self, rl='l', pose=1):
        """
        Grabs the needle with the specified gripper and specified pose.
        Pose descriptions are given in function moveNeedleToGripperPose.
        Does not check for collisions on doing so.
        """
        if self.sneedle is None:
            return
        
        if pose not in [1,2,3,4]:
            rospy.logwarn ("Unknown needle pose. Not grabbing needle.")
            return
        
        self.moveNeedleToGripperPose(rl,pose)
        
        oldManip = self.robot.GetActiveManipulator()
        manip = {'r':self.rarm.manip, 'l':self.larm.manip}[rl]
        self.robot.SetActiveManipulator(manip)
        
        grabbed = self.robot.Grab(self.sneedle)
        if grabbed is False:
            rospy.logwarn("Unable to grab the needle")
            self.resetNeedlePose()
        
        self.sneedle_pose = pose
        self.grabbingArm = {'r':self.rarm, 'l':self.larm}[rl]
        self.robot.SetActiveManipulator(oldManip)
        
    def releaseNeedle (self):
        """
        Releases the needle if it is grabbed. Returns needle to reset position.
        """
        if self.sneedle is None:
            return
        
        self.robot.Release(self.sneedle)
        self.resetNeedlePose()
        self.sneedle_pose = 0
        self.grabbingArm = None
        
    def needleTipTransform (self):
        """
        Returns the transform of the needle tip.
        Z is pointing out of the tip and X is pointing out of the plane.
        """
        needleTfm = self.sneedle.GetTransform()
        
        handleTrans = np.eye(4)
        handleTrans[[0,1],[3,3]] = np.array([0.07,0.05])
        handleRot = np.eye(4)
        ind1, ind2 = [0,0,1,1], [0,1,0,1]
        theta = -np.pi/6
        handleRot[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
        
        handleTfm = handleTrans.dot(handleRot)
        
        # Values found offline
        tipTrans = np.eye(4)
        tipTrans[[0,1],[3,3]] = np.array([-0.057,0.202])
        tipRot = np.eye(4)
        ind1, ind2 = [0,0,1,1], [0,1,0,1]
        theta = -2.43296373
        tipRot[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
        
        tipTfm = tipTrans.dot(tipRot)
        
        finalRot = np.eye(4)
        ind1, ind2 = [0,0,2,2], [0,2,0,2]
        theta = np.pi/2
        finalRot[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
        
        WfromTip = needleTfm.dot(handleTfm.dot(tipTfm.dot(finalRot)))
        
        return WfromTip
    
    def getGripperFromNeedleTipTransform(self):
        """
        Returns the transform between gripper and needle tip.
        """
        if not self.grabbingArm:
            rospy.logwarn('Needle not being held.')
            return None
        
        WfmNTip  = self.needleTipTransform()
        WfmEE    = self.grabbingArm.manip.GetTransform()
        EEfmNTip = nla.inv(WfmEE).dot(WfmNTip)
        
        return EEfmNTip
