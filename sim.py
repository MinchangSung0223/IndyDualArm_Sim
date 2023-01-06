import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import pi,sqrt,cos,sin
import pybullet_data
import numpy as np
import json
from modern_robotics import FKinSpace,IKinSpace,CartesianTrajectory,TransInv,Adjoint,JacobianBody,JacobianSpace,se3ToVec,MatrixLog6
np.set_printoptions(precision=4, suppress=True)
import math
import modern_robotics as mr
from mr_urdf_loader import loadURDF
from functions.utils import *
import matplotlib.pyplot as plt

def rotMat(theta,axis):
    R = np.eye(3)
    if axis == "x":
        R = np.array([[1 ,0 ,0],[0 ,cos(theta),-sin(theta)],[0 ,sin(theta) ,cos(theta)]])
    if axis == "y":
        R = np.array([[cos(theta),0 ,sin(theta)],[0 ,1,0],[-sin(theta),0,cos(theta)]])
    if axis == "z":
        R = np.array([[cos(theta),-sin(theta) ,0],[sin(theta) ,cos(theta),0],[0 ,0, 1]])
    return R

##MRdata load
file_path = "./MR_info.json"
data={}
with open(file_path, 'r') as file:
    data = json.load(file)
relativeSlist = np.array(data['relSlist'])
relativeBlist = np.array(data['relBlist'])
relativeM = np.array(data['relM'])
baseSlist = np.array(data['Slist'])
baseM = np.array(data['M'])
baseBlist = np.matmul(Adjoint(TransInv(baseM)),baseSlist)
Tb = np.array(data['Tbr'])
Tbl = np.array(data['Tbl'])

## pybullet env setup
urdf_name = "model/IndyDualArm.urdf"
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
CONTROL_FREQ = 240
timeStep = 1/CONTROL_FREQ
#p.setTimeStep(timeStep)
p.setPhysicsEngineParameter(fixedTimeStep = timeStep)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
robotId = p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
active_joint_list,active_joint_num_list,eef_num=getActiveJointList(robotId)

right_joint_list = [b'r_joint_0', b'r_joint_1', b'r_joint_2', b'r_joint_3', b'r_joint_4',b'r_joint_5']
right_joint_num_list = [2,3,4,5,6,7]
right_tcp = 8
left_joint_list = [b'l_joint_0', b'l_joint_1', b'l_joint_2', b'l_joint_3', b'l_joint_4', b'l_joint_5']
left_joint_num_list = [10, 11, 12, 13, 14, 15]
left_tcp = 16
D2R = np.pi/180.0

HomePos = [0*D2R ,-15*D2R,-90*D2R,0*D2R,-75*D2R,0*D2R,0*D2R ,15*D2R,90*D2R,0*D2R,75*D2R,0*D2R]
HomePos = [-0.146 ,-0.814, -1.308, -0.615 ,-1.030 ,0.070, 0.146 ,0.814, 1.308, 0.615 ,1.030 ,-0.070]
## reset joint state
for i in range(0,len(active_joint_num_list)):
	p.resetJointState(robotId,active_joint_num_list[i],HomePos[i])
	
## initialize all joint
initializeActiveJoints(robotId,right_joint_num_list)
initializeActiveJoints(robotId,left_joint_num_list)

MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
MAX_TORQUES_ALL = np.array([431.97,431.97,197.23,79.79,79.79,79.79,431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]

left_qddot = [0,0,0,0,0,0]
right_qddot = [0,0,0,0,0,0]


Xstart = np.eye(4)
Xstart[0:3,0:3] = np.array([[-1 ,0 ,0],[0 ,0 ,-1],[0 ,-1 ,0]])   
Xstart[0,3] = 0.5
Xstart[1,3] = -0.5
Xstart[2,3] = 0.3
Xend = np.eye(4)
Xend[0:3,0:3] =np.array([[-1 ,0 ,0],[0 ,0 ,-1],[0 ,-1 ,0]])  
Xend[0,3] = 0.5
Xend[1,3] = 0.5
Xend[2,3] = 0.3



endTime = 20;
dt = timeStep;
N =  int(endTime/dt);

traj = CartesianTrajectory(Xstart, Xend,endTime , N, 5)
traj = np.array(traj)


for t in np.arange(0,endTime,timeStep):
	left_q,left_qdot = getJointStates(robotId,left_joint_num_list)
	right_q,right_qdot = getJointStates(robotId,right_joint_num_list)	
	jointState = np.array(p.getJointStates(robotId,[2,3,4,5,6,7,10,11,12,13,14,15]))
	q = np.array(jointState[:,0])
	qdot = np.array(jointState[:,1])
	rel_q = [-right_q[5],-right_q[4],-right_q[3],-right_q[2],-right_q[1],-right_q[0],left_q[0],left_q[1],left_q[2],left_q[3],left_q[4],left_q[5]]
	relT = FKinSpace(relativeM,relativeSlist,np.array(rel_q).T)

	rightT = Tb @  FKinSpace(baseM,baseSlist,right_q)
	leftT =rightT @ relT
	
	print(relT)
	#rightT_= getEEFPose(robotId,8)
	#leftT_= getEEFPose(robotId,16)
	

	
	Jb_r = JacobianBody(baseBlist,right_q)
	Js_r = JacobianSpace(baseSlist,right_q)
	J2 = np.concatenate((-np.flip(Jb_r,1).T,np.zeros([6,6]))).T
	J1 = JacobianBody(relativeBlist,rel_q)
	#print(J1)
	targetT =  TransInv(Tb) @ traj[i,:,:]
	relative_targetT = np.array([[-1 ,0 ,0 ,0],[0,1 ,0 ,0],[0 ,0 ,-1 ,0.5],[0 ,0 ,0 ,1]])

	print(q)
	#print(relT)
	Xe_r = se3ToVec(MatrixLog6(TransInv(rightT)@ targetT))
	Xe_rel = se3ToVec(MatrixLog6(TransInv(relT)@ relative_targetT))
	Xe_rel_ = Xe_rel[3:6]
	pinvJ1 = np.linalg.pinv(J1);
	pinvJ2 = np.linalg.pinv(J2);
	J1_ = J1[3:6,:]
	pinvJ1_ = np.linalg.pinv(J1_);
	#print(J1.shape)
	rel_dq = 10*pinvJ1@ Xe_rel       
	rel_q = rel_q+rel_dq*timeStep;
	right_q_ = -np.flip(rel_q[0:6])
	left_q_ = rel_q[6:12]
	

	ID = p.calculateInverseDynamics(robotId,[q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11]],[qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5],qdot[6],qdot[7],qdot[8],qdot[9],qdot[10],qdot[11]],[0,0,0,0,0,0,0,0,0,0,0,0]);
	#setTorques(robotId, active_joint_num_list,ID,MAX_TORQUES_ALL)
	#setTorques(robotId, right_joint_num_list,right_ID,MAX_TORQUES)
	setPosition(robotId, right_joint_num_list,right_q_,MAX_TORQUES)	
	setPosition(robotId, left_joint_num_list,left_q_,MAX_TORQUES)		
	p.stepSimulation()
	time.sleep(timeStep);




