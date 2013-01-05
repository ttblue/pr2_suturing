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
        
        
        if initPos is not None:
            try:
                self.gotoArmPosture(initPos)
            except:
                print "Cannot go to pos ", initPos
                
        self.addTableToRave()
        
        
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
 