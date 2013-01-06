from IKPlannerFunctions import IKInterpolationPlanner
from brett2.PR2 import PR2, Arm, IKFail

import openravepy as opr
import numpy as np

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
        Moves the tool tip in the specified direction in the base_link frame. 
        
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
        

    def circleAroundRadius (self, d, rad, finAng, steps=10):
        """
        Moves the gripper in a circle.
        
        Direction of circle (either inner or outer)       -> d
        Radius of circle                                  -> rad
        Final angle covered by circle                     -> finAng
        Number of points of linear interpolation of angle -> steps
        """        
        self.pr2.update_rave()
        trajectory = self.planner.circleAroundRadius (d, rad, finAng)

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
    
    def cart_to_joint(self, matrix4, ref_frame, targ_frame, filter_options=-1):
        """
        Calculates IK for given transform between ref_frame and targ_frame. 
        """
        if filter_options==-1:
            filter_options = self.planner.filter_options
        self.pr2.update_rave()
        return Arm.cart_to_joint(self.manip, matrix4, ref_frame, targ_frame, filter_options)
        
        

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
        
        if initPos is not None:
            try:
                self.gotoArmPosture(initPos)
            except:
                print "Cannot go to pos ", initPos
                
        self.addTableToRave()
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

    def addNeedleToRave (self):
        """
        Adds the suturing needle from the .dae file.
        Needle is kept at the edge of the table to start with.
        """
        with self.env:
            self.sneedle = self.env.ReadKinBodyURI('/home/sibi/sandbox/bulletsim/data/needle/sneedle.dae')
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
            print "Unknown needle pose. Not setting any transform."
            return
                                        
        sndTfm = WfromEE.dot(rot.dot(trans))
        self.sneedle.SetTransform(sndTfm)
        
    def grabNeedle (self, rl='l', pose=1):
        """
        Grabs the needle with the specified gripper and specified pose.
        Pose descriptions are given in function moveNeedleToGripperPose.
        """
        if self.sneedle is None:
            return
        
        self.moveNeedleToGripperPose(rl,pose)
        
        oldManip = self.robot.GetActiveManipulator()
        manip = {'r':self.rarm.manip, 'l':self.larm.manip}[rl]
        self.robot.SetActiveManipulator(manip)
        
        grabbed = self.robot.Grab(self.sneedle)
        if grabbed is False:
            print "Unable to grab the needle"
            self.resetNeedlePose()
        
        self.robot.SetActiveManipulator(oldManip)
        
    def releaseNeedle (self):
        """
        Releases the needle if it is grabbed. Returns needle to reset position.
        """
        if self.sneedle is None:
            return
        
        self.robot.Release(self.sneedle)
        self.resetNeedlePose()