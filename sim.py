import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import pybullet as p
import time
from math import pi,sqrt,cos,sin
import pybullet_data
import numpy as np
import json
from modern_robotics import FKinSpace,IKinSpace,CartesianTrajectory,TransInv,Adjoint,JacobianBody,JacobianSpace,se3ToVec,MatrixLog6
file_path = "./MR_params.json"
data={}
with open(file_path, 'r') as file:
    data = json.load(file)
relativeSlist = np.reshape(np.array(data['relativeSlist']),(6,12))
relativeBlist = np.reshape(np.array(data['relativeBlist']),(6,12))
relativeM = np.reshape(np.array(data['relativeM']),(4,4))
baseSlist = np.reshape(np.array(data['baseSlist']),(6,6))
baseM = np.reshape(np.array(data['baseM']),(4,4))

Slist = np.reshape(np.array(data['Slist']),(6,6))
Blist = np.reshape(np.array(data['Blist']),(6,6))
M = np.reshape(np.array(data['M']),(4,4))

def rotMat(theta,axis):
	R = np.eye(3)
	if axis == "x":
		R = np.array([[1 ,0 ,0],[0 ,cos(theta),-sin(theta)],[0 ,sin(theta) ,cos(theta)]])
	if axis == "y":
		R = np.array([[cos(theta),0 ,sin(theta)],[0 ,1,0],[-sin(theta),0,cos(theta)]])
	if axis == "z":
		R = np.array([[cos(theta),-sin(theta) ,0],[sin(theta) ,cos(theta),0],[0 ,0, 1]])
	return R
np.set_printoptions(precision=4, suppress=True)
import math
import modern_robotics as mr
from mr_urdf_loader import loadURDF
from functions.utils import *

import matplotlib.pyplot as plt
urdf_name = "model/IndyDualArm.urdf"
## pybullet env setup
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
for i in range(0,len(active_joint_num_list)):
	p.resetJointState(robotId,active_joint_num_list[i],HomePos[i])

MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
MAX_TORQUES_ALL = np.array([431.97,431.97,197.23,79.79,79.79,79.79,431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
initializeActiveJoints(robotId,right_joint_num_list)
initializeActiveJoints(robotId,left_joint_num_list)

Tb = np.eye(4)
Tb[0:3,0:3] = rotMat(-np.pi/3,"x")
B_y = 0.15634
B_z = 0.37722
Tb[1,3] = B_y;
Tb[2,3] = B_z;
endTime = 300;
left_qddot = [0,0,0,0,0,0]
right_qddot = [0,0,0,0,0,0]
for t in np.arange(0,endTime,timeStep):
	left_q,left_qdot = getJointStates(robotId,left_joint_num_list)
	right_q,right_qdot = getJointStates(robotId,right_joint_num_list)	
	jointState = np.array(p.getJointStates(robotId,[2,3,4,5,6,7,10,11,12,13,14,15]))
	q = np.array(jointState[:,0])
	qdot = np.array(jointState[:,1])
	#print(left_q)
	rel_q = np.concatenate((-np.flip(right_q),left_q))
	relT = FKinSpace(relativeM,relativeSlist,rel_q)
	rightT = Tb@ FKinSpace(M,Slist,right_q)
	rightT_= getEEFPose(robotId,8)
	leftT = rightT@ relT
	leftT_= getEEFPose(robotId,16)
	print("leftT")
	print(leftT)
	print("leftT_")
	print(leftT_)
	ID = p.calculateInverseDynamics(robotId,[q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11]],[qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5],qdot[6],qdot[7],qdot[8],qdot[9],qdot[10],qdot[11]],[0,0,0,0,0,0,0,0,0,0,0,0]);
	#right_ID = p.calculateInverseDynamics(robotId,right_q,right_qdot,right_qddot);	
	
	setTorques(robotId, active_joint_num_list,ID,MAX_TORQUES_ALL)
	#setTorques(robotId, right_joint_num_list,right_ID,MAX_TORQUES)	
	p.stepSimulation()
	time.sleep(timeStep);


'''
## MR setup
MR=loadURDF(urdf_name)
M  = np.array(MR["M"])
Slist  = np.array(MR["Slist"])
Mlist  = np.array(MR["Mlist"])
Glist  = np.array(MR["Glist"])
Blist = np.array(MR["Blist"])
jointNum = MR["actuated_joints_num"]


## pybullet env setup
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
CONTROL_FREQ = 240
timeStep = 1/CONTROL_FREQ
#p.setTimeStep(timeStep)
p.setPhysicsEngineParameter(fixedTimeStep = timeStep)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.setRealTimeSimulation(True)
## robot setup
robotId = p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
active_joints_name,active_joints_num,eef_num=getActiveJointList(robotId)
initializeActiveJoints(robotId,active_joints_num)
## intialize 
g = np.array([0, 0,-9.8])
q = np.array([0,0,0,0,0,0])
qdot = np.array([0 ,0, 0, 0 ,0 ,0]);
qddot= np.array([0 ,0, 0, 0 ,0 ,0]);
MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
torques = np.array([0,0,0,0,0,0])
Ftip = np.array([0,0,0,0,0,0])

dq = np.array([1,1,1,1,1,1]) # desired q
dqdot = np.array([0.0 ,0.0, 0.0, 0.0 ,0.0 ,0.0]); # desired qdot
dqddot = np.array([0.0 ,0.0, 0.0, 0.0 ,0.0 ,0.0]); # desired qddot

Kp =  1000*np.diag([70,70,40,25,25,18]);
Kv=  np.diag([55,55,30,15,15,3]);

endTime = 3;

dq = mr.JointTrajectory(q, dq, endTime, int(endTime/timeStep), 5)

dataLogger = {"t":[],"dq":[],"q":[],"qdot":[],"e":[],"edot":[],"torques":[]}

print(dq.shape)
n = 0
for t in np.arange(0,endTime,timeStep):
	q,qdot = getJointStates(robotId,active_joints_num)
	MassMatrix =mr.MassMatrix(q,Mlist,  Glist, Slist)
	MRID =mr.InverseDynamics(q, qdot, qddot, g, Ftip, Mlist,  Glist, Slist) #inverse dynamics ,ID =  C(qdot)* qdot + G(q)

	e = dq[n,:]- q
	edot = dqdot-qdot
	qddot_ref = dqddot+Kv @  edot + Kp @ e
	torques =MassMatrix @  qddot_ref + MRID
	setTorques(robotId,active_joints_num,torques,MAX_TORQUES)


	dataLogger["t"].append(t)
	dataLogger["q"].append(q)
	dataLogger["dq"].append(dq[n,:])	
	dataLogger["qdot"].append(qdot)
	dataLogger["e"].append(e)
	dataLogger["edot"].append(edot)		
	dataLogger["torques"].append(torques)		
				
	n = n+1
	p.stepSimulation()
	


for i in range(0,jointNum):
	plt.subplot(jointNum, 1, i+1)  
	plt.plot(dataLogger["t"],np.array(dataLogger["q"])[:,i],'r')
	plt.plot(dataLogger["t"],np.array(dataLogger["dq"])[:,i],'b:')
	plt.plot(dataLogger["t"],np.array(dataLogger["e"])[:,i],'k--')
		
plt.show()
'''

