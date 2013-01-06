import rospy
import numpy as np
from ArmPlannerPR2 import PlannerPR2

def func (x):
    rospy.sleep(x)
    p.lgrip.close()

def initVec (tfm):
    vx, vy, vz = tfm[0:3,0], tfm[0:3,1], tfm[0:3,2]
    return vx, vy, vz

def addVec (vec):
    sndTfm[0:3,3] += np.unwrap(vec)
    snd.SetTransform(sndTfm)

def rotate (theta,d='x'):
    global sndTfm
    if d=='x':
        ind1, ind2 = [1,1,2,2], [1,2,1,2]
    elif d=='y':
        ind1, ind2 = [0,0,2,2], [0,2,0,2]
    elif d=='z':
        ind1, ind2 = [0,0,1,1], [0,1,0,1]
        
    rot = np.eye(4)
    rot[ind1,ind2] = np.array([np.cos(theta), -1*np.sin(theta), np.sin(theta), np.cos(theta)])
    sndTfm = sndTfm.dot(rot)
    snd.SetTransform(sndTfm) 
    
if __name__ == "__main__":
    rospy.init_node("Test_Planning")
    p = PlannerPR2 ()
    env = p.env
    pi = np.pi

    sndTfm = p.larm.manip.GetTransform() 
    x,y,z = initVec(sndTfm)

    env.SetViewer('qtcoin')
