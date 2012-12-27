from brett2.PR2 import PR2
import openravepy as orpy
import numpy as np

class IKInterpolationPlanner(object):
    
    def __init__(self, _pr2, _rl):
        self.pr2 = _pr2
        self.rl = _rl
        self.arm = {'l':self.pr2.larm, 'r':self.pr2.rarm}[self.rl]
        
    def plan(self, transforms):
        if len(transforms) < 1:
            AssertionError("Not enough transforms!")
        
        if len(transforms) == 1:
            firstTransform = self.arm.manip.GetEndEffectorTransform()
            transforms = [firstTransform transforms[0]]