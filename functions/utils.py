import pybullet as p
import numpy as np
np.set_printoptions(precision=4, suppress=True)
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from modern_robotics import *
import matplotlib.pyplot as plt
import json
import pybullet_data
D2R = np.pi/180.0
right_joint_list = [b'r_joint_0', b'r_joint_1', b'r_joint_2', b'r_joint_3', b'r_joint_4',b'r_joint_5']
right_joint_num_list = [2,3,4,5,6,7]
right_tcp = 8
left_joint_list = [b'l_joint_0', b'l_joint_1', b'l_joint_2', b'l_joint_3', b'l_joint_4', b'l_joint_5']
left_joint_num_list = [10, 11, 12, 13, 14, 15]
left_tcp = 16	
active_joint_num_list=[2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]
MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
MAX_TORQUES_ALL = np.array([431.97,431.97,197.23,79.79,79.79,79.79,431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
HomePos = [-0.146 ,-0.814, -1.308, -0.615 ,-1.030 ,0.070, 0.146 ,0.814, 1.308, 0.615 ,1.030 ,-0.070]

def PybulletSetup(p,urdf_name = "model/IndyDualArm.urdf", CONTROL_FREQ = 240.0):
	## pybullet env setup
	p.connect(p.GUI)
	p.setGravity(0, 0, -9.8)
	p.setRealTimeSimulation(True)
	timeStep = 1/CONTROL_FREQ
	p.setTimeStep(timeStep)
	p.setPhysicsEngineParameter(fixedTimeStep = timeStep)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	robotId = p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=p.URDF_USE_SELF_COLLISION)
	planeId = p.loadURDF("plane.urdf",[0, 0, -0.521])
	p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
	p.resetDebugVisualizerCamera(2,90,-15,[0,0,0.5])
	p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
	active_joint_list,active_joint_num_list,eef_num=getActiveJointList(robotId)
	## initialize all joint
	initializeActiveJoints(robotId,right_joint_num_list)
	initializeActiveJoints(robotId,left_joint_num_list)
	dt = 1.0/CONTROL_FREQ
	return right_joint_num_list,left_joint_num_list,active_joint_num_list[0:12],robotId,dt
def rotMat(theta,axis):
    R = np.eye(3)
    if axis == "x":
        R = np.array([[1 ,0 ,0],[0 ,cos(theta),-sin(theta)],[0 ,sin(theta) ,cos(theta)]])
    if axis == "y":
        R = np.array([[cos(theta),0 ,sin(theta)],[0 ,1,0],[-sin(theta),0,cos(theta)]])
    if axis == "z":
        R = np.array([[cos(theta),-sin(theta) ,0],[sin(theta) ,cos(theta),0],[0 ,0, 1]])
    return R

def MRSetup(file_path):
	##MRdata load
	data={}
	with open(file_path, 'r') as file:
	    data = json.load(file)
	relSlist = np.array(data['relSlist'])
	relBlist = np.array(data['relBlist'])
	relM = np.array(data['relM'])
	baseSlist = np.array(data['baseSlist'])
	baseM = np.array(data['baseM'])
	baseBlist = np.array(data['baseBlist'])
	Tb = np.array(data['Tbr'])
	return baseSlist,baseBlist,baseM,relSlist,relBlist,relM,Tb

def getJacobian(Slist,Blist,thetalist):
	thetalist = np.reshape(thetalist,(len(thetalist),1))
	Js = JacobianSpace(Slist,thetalist);
	Jb = JacobianSpace(Blist,thetalist);
	return Js,Jb,pinv(Js),pinv(Jb)
def reset_q_rel(robotId,q_rel):
	q_rel_ = [-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0],q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11]]
	for i in range(0,len(q_rel_)):
		p.resetJointState(robotId,active_joint_num_list[i],q_rel_[i])
def reset_q(robotId,q):
	for i in range(0,len(q)):
		p.resetJointState(robotId,active_joint_num_list[i],q[i])

def reset_q_r(q_r):
	for i in range(0,len(right_joint_num_list)):
		p.resetJointState(robotId,right_joint_num_list[i],q_r[i])

def reset_q_l(q_l):
	for i in range(0,len(left_joint_num_list)):
		p.resetJointState(robotId,left_joint_num_list[i],q_l[i])

def get_q_rel(q_r,q_l):
	q_rel = [-q_r[5],-q_r[4],-q_r[3],-q_r[2],-q_r[1],-q_r[0],q_l[0],q_l[1],q_l[2],q_l[3],q_l[4],q_l[5]]
	return np.array(q_rel).T;
def get_qdot_rel(qdot_r,qdot_l):
	qdot_rel = [-qdot_r[5],-qdot_r[4],-qdot_r[3],-qdot_r[2],-qdot_r[1],-qdot_r[0],qdot_l[0],qdot_l[1],qdot_l[2],qdot_l[3],qdot_l[4],qdot_l[5]]
	return np.array(qdot_rel).T;	
def get_q_r(q_rel):
	return np.array([-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0]]).T
def get_q_l(q_rel):
	return np.array([q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11]]).T	
def getJointStates(robotId,j_list):
	jointState = p.getJointStates(robotId,j_list)
	q=[]
	qdot=[]
	for i in range(0,len(j_list)):
		js = jointState[i];
		q.append(js[0])
		qdot.append(js[1])
	q = np.array(q)
	qdot = np.array(qdot)
	return q,qdot
def getEEFPose(robotId,eef_num):
	Teef = np.eye(4);
	ret = p.getLinkState(robotId,eef_num,1,1)
	pos = ret[0]
	orn = ret[1]	
	Teef[0:3,3] = np.array(pos).T
	Teef[0:3,0:3] = np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
	return Teef	
def getActiveJointList(robotId):
	active_joint_list=[]
	active_joint_num_list=[]	
	numJoint = p.getNumJoints(robotId)
	print("============================JOINT_INFO====================================")
	for i in range(0,numJoint):
		info = p.getJointInfo(robotId,i)
		joint_type = info[2]
		joint_type_name =""
		if joint_type == p.JOINT_FIXED:
			joint_type_name = "JOINT_FIXED"
		elif joint_type == p.JOINT_REVOLUTE:
			joint_type_name = "JOINT_REVOLUTE"
		elif joint_type == p.JOINT_PRISMATIC:
			joint_type_name = "JOINT_PRISMATIC"
		elif joint_type == p.JOINT_SPHERICAL:
			joint_type_name = "JOINT_SPHERICAL"
		elif joint_type == p.JOINT_PLANAR:
			joint_type_name = "JOINT_PLANAR"
											
		print("Joint Num : ", info[0] , "\t Joint Name : ", info[1], "\t Joint Type : " , joint_type_name)
		if info[2] !=  p.JOINT_FIXED:
			active_joint_list.append(info[1])
			active_joint_num_list.append(info[0])

	print("=========================================================================")		
	eef_num = numJoint-1;
	return	active_joint_list,active_joint_num_list,eef_num
def getControlVelocity(Kp,Kd,Ki,Xe,Xedot):
	print("d")

def initializeActiveJoints(robotId,active_joints_num):
	for i in active_joints_num:
		p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)	
def setTorques(robotId, active_joints_num,torques,MAX_TORQUES):
	idx =0
	def saturation(x,max_x):
		if x>= 0 : return np.min([x,max_x])
		else : return np.max([x,-max_x])
	actuated_torques = []
	for i in active_joints_num:
		p.setJointMotorControl2(robotId, i, p.TORQUE_CONTROL, force=saturation(torques[idx],MAX_TORQUES[idx]))	
		actuated_torques.append(saturation(torques[idx],MAX_TORQUES[idx]))
		
		idx=idx+1
	#print("input torques : ", torques)
	#print("actuated_torques : ", actuated_torques)
def saveData(data,file_path):
	for val in data:
		data[val] =np.array(data[val]).tolist();
	with open(file_path, 'w') as outfile:
		json.dump(data, outfile)
def drawData(file_path):
	data={}
	with open(file_path, 'r') as file:
	    data = json.load(file)
	t_list = np.array(data["t"])
	q_rel_list = np.array(data["q_rel"])

	T_rel_list = np.array(data["T_rel"])
	T_rel_x_list = T_rel_list[:,0,3]
	T_rel_y_list = T_rel_list[:,1,3]
	T_rel_z_list = T_rel_list[:,2,3]

	Xd_list = np.array(data["Xd"])
	Xd_x_list = Xd_list[:,0,3]
	Xd_y_list = Xd_list[:,1,3]
	Xd_z_list = Xd_list[:,2,3]
	#for i in range(0,12):
	#	plt.plot(t_list,q_rel_list[:,i])
	plt.subplot(3,1,1)
	plt.plot(t_list,T_rel_x_list)
	plt.plot(t_list,Xd_x_list)
	plt.subplot(3,1,2)
	plt.plot(t_list,T_rel_y_list)
	plt.plot(t_list,Xd_y_list)
	plt.subplot(3,1,3)
	plt.plot(t_list,T_rel_z_list)
	plt.plot(t_list,Xd_z_list)
	plt.tight_layout()
	plt.show()
	

def setPosition(robotId, active_joints_num,q_rel,MAX_TORQUES_ALL):
	idx =0
	q_rel_ = [-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0],q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11]]
	for i in active_joints_num:
		p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL,targetPosition=q_rel_[idx], force=MAX_TORQUES_ALL[idx])	
		idx=idx+1