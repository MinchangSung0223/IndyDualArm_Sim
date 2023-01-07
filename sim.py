import pybullet as p
import time
from math import *
from modern_robotics import *
from functions.utils import *

baseSlist,baseBlist,baseM,relSlist,relBlist,relM,Tb=MRSetup("./MR_info.json");
r_j_list,l_j_list,a_j_list,robotId,dt=PybulletSetup(p,urdf_name = "model/IndyDualArm.urdf", CONTROL_FREQ = 240.0)
reset_q(robotId,HomePos)
endTime = 5;

Orn_Kp = 20.0/30.0;
Orn_Kd = Orn_Kp/100.0;
Orn_Ki = Orn_Kp/100.0;

Pos_Kp = 20.0
Pos_Kd = Pos_Kp/50.0;
Pos_Ki = Pos_Kp/100.0;
Xe_rel_sum = np.array([0,0,0,0,0,0]).T
data={}
data["t"]=[]
data["q_rel"]=[]
data["T_rel"]=[]
data["Xd"]=[]
q_l,qdot_l = getJointStates(robotId,l_j_list)
q_r,qdot_r = getJointStates(robotId,r_j_list)	
q_rel = get_q_rel(q_r,q_l)
T_rel = FKinSpace(relM,relSlist,q_rel)
T_rel_end  = T_rel
T_rel_end[0,3] = T_rel_end[0,3]+0.2
T_rel_end[1,3] = T_rel_end[1,3]-0.2
T_rel_end[2,3] = T_rel_end[2,3]-0.2
T_rel = FKinSpace(relM,relSlist,q_rel)
Xd_list = CartesianTrajectory(T_rel, T_rel_end, endTime, int(endTime/dt), 5)
idx =0
for t in np.arange(0,endTime,dt):
	q_l,qdot_l = getJointStates(robotId,l_j_list)
	q_r,qdot_r = getJointStates(robotId,r_j_list)	
	q,qdot = getJointStates(robotId,a_j_list)
	q_rel = get_q_rel(q_r,q_l)
	qdot_rel = get_qdot_rel(qdot_r,qdot_l);
	T_rel = FKinSpace(relM,relSlist,q_rel)
	T_r = Tb @  FKinSpace(baseM,baseSlist,q_r)
	T_l =T_r @ T_rel
	Js_r,Jb_r,pinvJs_r,pinvJb_r=getJacobian(baseSlist,baseBlist,q_r)
	Js_rel,Jb_rel,pinvJs_rel,pinvJb_rel = getJacobian(relSlist,relBlist,q_rel)
	Xd = np.array([[-1 ,0 ,0 ,0],[0,1 ,0 ,0],[0 ,0 ,-1 ,0.4],[0 ,0 ,0 ,1]])
	#Xd = Xd_list[idx]
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


	Xe_rell_All = Kp_Xe_rel+Kd_Xe_rel+Ki_Xe_rel
	#qdot_rel = 20*pinvJb_rel[:,3:6] @ Xe_rel[3:6]
	qdot_rel = 20*pinvJb_rel @ Xe_rell_All
	q_rel,qdot_rel=EulerStep(q_rel,qdot_rel,np.zeros((12,1)),dt)	
	q_r=get_q_r(q_rel);
	q_l=get_q_l(q_rel);
	setPosition(robotId, a_j_list,q_rel,MAX_TORQUES_ALL)
	print(t)		
	data["t"].append(t)
	data["q_rel"].append(q_rel)
	data["T_rel"].append(T_rel)
	data["Xd"].append(Xd)
	idx = idx+1
	p.stepSimulation()
saveData(data,"output.json")	
drawData("output.json")	


