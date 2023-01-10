import pybullet as p
import time
from math import *
from modern_robotics import *
from functions.Indy7  import Indy7

CONTROL_FREQ = 240.0
endTime = 20;
dt = 1/CONTROL_FREQ;

indy7 = Indy7(p,urdf_name = "model/indy7.urdf", CONTROL_FREQ=CONTROL_FREQ)
baseSlist,baseBlist,baseM,relSlist,relBlist,relM,Tb=indy7.MRSetup("./MR_info.json");
indy7.resetJointStates(np.array([0,0,-pi/2.0,0,-pi/2.0,0]).T)
indy7.setJointStates(np.array([0,0,-pi/2.0,0,-pi/2.0,0]).T)
indy7.step()
Xd = np.array([[-1.   ,  -0.   ,  -0.  ,    0.35  ],
 [-0.    ,  1.,      0. ,    -0.2],
 [ 0.  ,    0.,     -1.   ,   0.53],
 [ 0.   ,   0. ,     0.   ,   1.    ]])

q,qdot = indy7.getJointStates();

print(baseSlist)
print(baseBlist)
print(Xd)

for t in np.arange(0,endTime,dt):

	Js,Jb,pinvJs,pinvJb = indy7.getJacobian(baseSlist,baseBlist,q)
	print(Jb)
	T = FKinSpace(baseM,baseSlist,q);
	T_ = indy7.getEEFPose();
	tau = indy7.getActuatedTorques();

	Xe = se3ToVec(MatrixLog6(TransInv(T)@ Xd));
	qdot = (pinvJb+np.eye(6)*0.001) @ Xe;
	q,qdot=EulerStep(q,qdot,np.zeros((6,1)),dt)
	indy7.setJointStates(q)


	#print(targetT)
	#indy7.setExternalForce([0,-10,-10])
	#F = indy7.getFT();
	#print(F)
	#print(tau)
	#print(np.linalg.inv(Jb.T)@tau)
	indy7.step()