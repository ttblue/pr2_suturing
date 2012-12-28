import rospy
from ArmPlannerPR2 import PlannerPR2
from markers import MarkerPlacer

if __name__ == "__main__":
    rospy.init_node("Test_Planning")
    
    mPlacer = MarkerPlacer()
    p = PlannerPR2 ()
    p.larm.goInWorldDirection('b',0.2)
    p.larm.circleAroundRadius(1,0.1,1.5,markerPlacer=mPlacer)