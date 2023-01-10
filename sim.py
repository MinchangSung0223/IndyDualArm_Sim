import pybullet as p
import time
from math import *
from modern_robotics import *
from functions.IndyDualArm  import IndyDualArm
CONTROL_FREQ = 240.0
endTime = 5;
dt = 1/CONTROL_FREQ;

indy = IndyDualArm(p,urdf_name = "model/IndyDualArm.urdf", CONTROL_FREQ=CONTROL_FREQ)
baseSlist,baseBlist,baseM,relSlist,relBlist,relM,Tb=indy.MRSetup("./MR_info.json");

indy.resetJointStates(indy.HomePosRel)
Orn_Kp = 20.0/30.0;
Orn_Kd = Orn_Kp/50.0;
Orn_Ki = Orn_Kp/100.0;

Pos_Kp = 20.0
Pos_Kd = Pos_Kp/50.0;
Pos_Ki = Pos_Kp/100.0;
Xe_rel_sum = np.array([0,0,0,0,0,0]).T
lineWidth = 1
colorRGB = [1, 0, 0]



for t in np.arange(0,endTime,dt):
	q_r,qdot_r=indy.getRightJointStates()
	q_l,qdot_l=indy.getLeftJointStates()
	q_rel = indy.get_q_rel(q_r,q_l)
	qdot_rel = indy.get_qdot_rel(qdot_r,qdot_l);
	T_rel = FKinSpace(relM,relSlist,q_rel)
	T_r = Tb @  FKinSpace(baseM,baseSlist,q_r)
	T_r_ = FKinSpace(baseM,baseSlist,q_r);
	T_l =T_r @ T_rel
	Js_r,Jb_r,pinvJs_r,pinvJb_r=indy.getJacobian(baseSlist,baseBlist,q_r)
	Js_rel,Jb_rel,pinvJs_rel,pinvJb_rel=indy.getJacobian(relSlist,relBlist,q_rel)
	Xd = np.array([[-1 ,0 ,0 ,0],[0,1 ,0 ,0],[0 ,0 ,-1 ,0.5*exp(-t)],[0 ,0 ,0 ,1]])

	Xe_rel = se3ToVec(MatrixLog6(TransInv(T_rel)@ Xd))
	V_rel = Jb_rel@qdot_rel
	Xedot_rel = Adjoint(TransInv(T_rel)@ Xd) @ np.array([0,0,0,0,0,0]).T -V_rel
	Xe_rel_sum = Xe_rel_sum+Xe_rel*dt;

	Kp_Xe_rel = Xe_rel
	Kp_Xe_rel[0:3] = Orn_Kp*Xe_rel[0:3] 
	Kp_Xe_rel[3:6] = Pos_Kp*Xe_rel[3:6] 
	Ki_Xe_rel = Xe_rel_sum
	Ki_Xe_rel[0:3] = Orn_Ki*Xe_rel_sum[0:3] 
	Ki_Xe_rel[3:6] = Pos_Ki*Xe_rel_sum[3:6] 
	Kd_Xe_rel = Xedot_rel
	Kd_Xe_rel[0:3] = Orn_Kd*Xedot_rel[0:3] 
	Kd_Xe_rel[3:6] = Pos_Kd*Xedot_rel[3:6] 

	Xe_rell_All = Kp_Xe_rel+Kd_Xe_rel+np.tanh(Ki_Xe_rel)
	qdot_rel = 20*pinvJb_rel @ Xe_rell_All
	q_rel,qdot_rel=EulerStep(q_rel,qdot_rel,np.zeros((12,1)),dt)	

	F_r = indy.getRightFT();
	F_l = indy.getLeftFT();
	tau_r = indy.getRightActuatedTorques();
	tau_l = indy.getRightActuatedTorques();
	indy.showContacts();
	indy.setData(t,q_rel,T_rel,Xd,F_r,F_l)
	indy.setJointStates(q_rel)
	indy.step();
	#time.sleep(0.001)

p.stopStateLogging(indy.logId)
p.disconnect()
#indy.saveData(indy.data,"output.json")	
#indy.drawData("output.json")	

