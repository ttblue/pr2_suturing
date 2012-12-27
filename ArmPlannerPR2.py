from IKPlannerFunctions import IKInterpolationPlanner
from brett2.PR2 import PR2, Arm, IKFail

"""
Planner class for the Arm.
"""
class PlannerArm(Arm):

    def __init__ (self, pr2, rl):
        Arm.__init__(self, pr2, rl)
        self.planner = IKInterpolationPlanner(self, self.lr)

    """
    Moves the tool tip in the specified direction in the gripper frame.
     
    Direction of movement                    -> dir
        f -> forward (along the tip of the gripper)
        b -> backward
        u -> up
        d -> down
        l -> left
        r -> right
    Distance traveled by tool tip            -> dist
    Number of points of linear interpolation -> steps
    """
    def goInDirection (self, dir, dist, steps=10):
        self.pr2.update_rave()
        trajectory = self.planner.goInDirection(dir, dist,steps)
        
        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail

    """
    Moves the tool tip in the specified direction in the base_link frame. 
    
    Direction of movement                    -> dir
        f -> forward
        b -> backward
        u -> up
        d -> down
        l -> left
        r -> right
    Distance traveled by tool tip            -> dist
    Number of points of linear interpolation -> steps
    """
    def goInWorldDirection (self, dir, dist, steps=10):
        self.pr2.update_rave()
        trajectory = self.planner.goInWorldDirection(dir, dist,steps)
        
        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail
        
    """
    Moves the gripper in a circle.
    
    Direction of circle (either inner or outer)       -> dir
    Radius of circle                                  -> rad
    Final angle covered by circle                     -> finAng
    Number of points of linear interpolation of angle -> steps
    """
    def circleAroundRadius (self, dir, rad, finAng, steps=10):
        self.pr2.update_rave()
        trajectory = self.planner.circleAroundRadius (dir, rad, finAng)

        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail
        
        
"""
Planner class for PR2 with planning arms.
"""
class PlannerPR2 (PR2):
    
    def __init__ (self):
        PR2.__init__(self)
        self.rarm = PlannerArm(self,'r')
        self.larm = PlannerArm(self, 'l')
        
    