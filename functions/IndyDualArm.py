import pybullet as p
import numpy as np
np.set_printoptions(precision=4, suppress=True)
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from modern_robotics import *
import matplotlib.pyplot as plt
from math import *
import json
import pybullet_data
import struct

class IndyDualArm:
    """
    IndyDualArm class
    """
    def __init__(self,p,urdf_name = "model/IndyDualArm.urdf", CONTROL_FREQ = 240.0):
        self.right_joint_list = [b'r_joint_0', b'r_joint_1', b'r_joint_2', b'r_joint_3', b'r_joint_4',b'r_joint_5']
        self.right_joint_num_list = [2,3,4,5,6,7]
        self.right_tcp = 8
        self.left_joint_list = [b'l_joint_0', b'l_joint_1', b'l_joint_2', b'l_joint_3', b'l_joint_4', b'l_joint_5']
        self.left_joint_num_list = [10, 11, 12, 13, 14, 15]
        self.left_tcp = 16   
        self.active_joint_num_list=[2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]
        self.MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
        self.MAX_TORQUES_ALL = np.array([431.97,431.97,197.23,79.79,79.79,79.79,431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
        self.HomePos = [-0.146 ,-0.814, -1.308, -0.615 ,-1.030 ,0.070, 0.146 ,0.814, 1.308, 0.615 ,1.030 ,-0.070]
        self.HomePosRel = [-0.070,1.030,0.615,1.308,0.814,0.146, 0.146 ,0.814, 1.308, 0.615 ,1.030 ,-0.070]
        self.p = p
        self.p.connect(self.p.GUI)
        #self.p.connect(self.p.GUI,options="--width=1920 --height=1080 --mp4=data.mp4")
        self.p.setGravity(0, 0, -9.8)
        self.p.setRealTimeSimulation(0)
        timeStep = 1/CONTROL_FREQ;
        self.p.setTimeStep(timeStep)
        self.p.setPhysicsEngineParameter(fixedTimeStep = timeStep)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robotId = self.p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=self.p.URDF_USE_SELF_COLLISION)
        self.planeId = self.p.loadURDF("plane.urdf",[0, 0, -0.521])
        self.boxId = self.p.loadURDF("model/box.urdf",[0.35, 0, 0.1])
        
        self.logId=p.startStateLogging(self.p.STATE_LOGGING_GENERIC_ROBOT,"data.bin",[self.robotId],maxLogDof=24)

        self.p.configureDebugVisualizer(self.p.COV_ENABLE_GUI,0)
        self.p.resetDebugVisualizerCamera(1.5,90,-15,[0,0,0.5])
        self.p.resetBasePositionAndOrientation(self.robotId, [0, 0, 0], [0, 0, 0, 1])
        active_joint_list,active_joint_num_list,eef_num=self.getActiveJointList(self.robotId)
        self.p.enableJointForceTorqueSensor(self.robotId,self.right_tcp,True)
        self.p.enableJointForceTorqueSensor(self.robotId,self.left_tcp,True)
        ## initialize all joint
        self.initializeActiveJoints(self.robotId,self.right_joint_num_list)
        self.initializeActiveJoints(self.robotId,self.left_joint_num_list)

        self.data={}
        self.data["t"]=[]
        self.data["q_rel"]=[]
        self.data["T_rel"]=[]
        self.data["Xd"]=[]
        self.data["F_r"]=[]
        self.data["F_l"]=[]
    
        self.lineId_list = []
        for i in range(0,17):
            lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                                    lineToXYZ=[0, 0, 0],
                                    lineColorRGB=[1,0,0],
                                    lineWidth=1,
                                    lifeTime=0)
            self.lineId_list.append(lineId)

        dt = 1.0/CONTROL_FREQ

    def getActiveJointList(self,robotId):
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
        return  active_joint_list,active_joint_num_list,eef_num        
    def initializeActiveJoints(sel,robotId,active_joints_num):
        for i in active_joints_num:
            p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)              
    def MRSetup(sel,file_path):
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

    def getJointStates(self,j_list):
        jointState = p.getJointStates(self.robotId,j_list)
        q=[]
        qdot=[]
        for i in range(0,len(j_list)):
            js = jointState[i];
            q.append(js[0])
            qdot.append(js[1])
        q = np.array(q)
        qdot = np.array(qdot)
        return q,qdot
    def getRightJointStates(self):
        q_r,qdot_r = self.getJointStates(self.right_joint_num_list)
        return q_r,qdot_r
    def getLeftJointStates(self):
        q_l,qdot_l =self.getJointStates(self.left_joint_num_list)
        return q_l,qdot_l
    def setJointStates(self,q_rel):
        idx =0
        q_rel_ = [-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0],q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11]]
        for i in self.active_joint_num_list:
            self.p.setJointMotorControl2(self.robotId, i, self.p.POSITION_CONTROL,targetPosition=q_rel_[idx], force=self.MAX_TORQUES_ALL[idx])  
            idx=idx+1
    def setJointStates(self,q_rel):
        idx =0
        q_rel_ = [-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0],q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11]]
        for i in self.active_joint_num_list:
            self.p.setJointMotorControl2(self.robotId, i, self.p.POSITION_CONTROL,targetPosition=q_rel_[idx], force=self.MAX_TORQUES_ALL[idx])  
            idx=idx+1

    def resetJointStates(self,q_rel):
        q_rel_ = [-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0],q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11]]
        for i in range(0,len(q_rel_)):
            self.p.resetJointState(self.robotId,self.active_joint_num_list[i],q_rel_[i])
    def get_q_rel(self,q_r,q_l):
        q_rel = [-q_r[5],-q_r[4],-q_r[3],-q_r[2],-q_r[1],-q_r[0],q_l[0],q_l[1],q_l[2],q_l[3],q_l[4],q_l[5]]
        return np.array(q_rel).T;
    def get_q_rel(self,q_r,q_l):
        q_rel = [-q_r[5],-q_r[4],-q_r[3],-q_r[2],-q_r[1],-q_r[0],q_l[0],q_l[1],q_l[2],q_l[3],q_l[4],q_l[5]]
        return np.array(q_rel).T;        
    def get_q_rl(self,q_rel):
        q_r = np.array([-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0]]).T
        q_l = np.array([q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_l[11]]).T
        return q_r,q_l;
    def setData(self,t,q_rel,T_rel,Xd,F_r,F_l):
        self.data["t"].append(t)
        self.data["q_rel"].append(q_rel)
        self.data["T_rel"].append(T_rel)
        self.data["Xd"].append(Xd)
        self.data["F_r"].append(F_r)
        self.data["F_l"].append(F_l)
    def saveData(self,data,file_path):
        for val in data:
            data[val] =np.array(data[val]).tolist();
        with open(file_path, 'w') as outfile:
            json.dump(data, outfile)
    def drawData(self,file_path):
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
        #   plt.plot(t_list,q_rel_list[:,i])
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
    
    def getJacobian(self,Slist,Blist,thetalist):
        thetalist = np.reshape(thetalist,(len(thetalist),1))
        Js = JacobianSpace(Slist,thetalist);
        Jb = JacobianSpace(Blist,thetalist);
        return Js,Jb,pinv(Js),pinv(Jb)
    def get_qdot_rel(self,qdot_r,qdot_l):
        qdot_rel = [-qdot_r[5],-qdot_r[4],-qdot_r[3],-qdot_r[2],-qdot_r[1],-qdot_r[0],qdot_l[0],qdot_l[1],qdot_l[2],qdot_l[3],qdot_l[4],qdot_l[5]]
        return np.array(qdot_rel).T;    
    def getRightFT(self):
        val = np.array(p.getJointState(self.robotId,self.right_tcp)[2]).T
        norm_val =sqrt(val[3]*val[3]+val[4]*val[4]+val[5]*val[5])
        return np.array([val[3],val[4],val[5],val[0],val[1],val[2]]).T
    def getLeftFT(self):
        val = np.array(p.getJointState(self.robotId,self.left_tcp)[2]).T
        norm_val =sqrt(val[3]*val[3]+val[4]*val[4]+val[5]*val[5])
        return np.array([val[3],val[4],val[5],val[0],val[1],val[2]]).T
    def getRightActuatedTorques(self):
        actuated_torques=[]
        for i in range(len(self.right_joint_num_list)):
            tempState = p.getJointState(self.robotId,self.right_joint_num_list[i])
            actuated_torques.append(tempState[3])
        return np.array(actuated_torques)
    def getLeftActuatedTorques(self):
        actuated_torques=[]
        for i in range(len(self.left_joint_num_list)):
            tempState = p.getJointState(self.robotId,self.left_joint_num_list[i])
            actuated_torques.append(tempState[3])
        return np.array(actuated_torques)

    def getRightEEFPose(self):
        Teef = np.eye(4);
        ret = p.getLinkState(self.robotId,self.right_tcp,1,1)
        pos = ret[0]
        orn = ret[1]    
        Teef[0:3,3] = np.array(pos).T
        Teef[0:3,0:3] = np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
        return Teef 
    def getLeftEEFPose(self):
        Teef = np.eye(4);
        ret = p.getLinkState(self.robotId,self.left_tcp,1,1)
        pos = ret[0]
        orn = ret[1]    
        Teef[0:3,3] = np.array(pos).T
        Teef[0:3,0:3] = np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
        return Teef 
    def step(self)      :
        #self.t += self.timeStep
        self.p.stepSimulation()
    def showContacts(self):        
        closest_pts = self.p.getClosestPoints(bodyA=self.planeId,bodyB=self.robotId,distance=10,collisionShapeA=-1,collisionShapeB=self.left_tcp)
        contact_pts = self.p.getContactPoints()
        if len(contact_pts)>0:
            distance = contact_pts[0][8]
            ptA = contact_pts[0][5]
            ptB = contact_pts[0][6]
            contactDistance =contact_pts[0][8]
            real_contactNormalForce=contact_pts[0][9]
            contactNormalForce=contact_pts[0][9]/5000.0 # drawing
            normalBtoA = contact_pts[0][7]
            #print(contactNormalForce)
            def map(minVal,maxVal,val):
                return (val-minVal)/(maxVal-minVal);
            p.addUserDebugLine(lineFromXYZ=ptA,
                               lineToXYZ=[ptA[0]+normalBtoA[0]*contactNormalForce, ptA[1]+normalBtoA[1]*contactNormalForce,ptA[2]+normalBtoA[2]*contactNormalForce],
                               lineColorRGB=[1,1-map(0,0.5,contactNormalForce),0],
                               lineWidth=100,
                               lifeTime=100)
            '''
            p.addUserDebugLine(lineFromXYZ=ptB,
                               lineToXYZ=[ptB[0]-normalBtoA[0]*contactNormalForce, ptB[1]-normalBtoA[1]*contactNormalForce,ptB[2]-normalBtoA[2]*contactNormalForce],
                               lineColorRGB=[1,1-map(0,0.5,contactNormalForce),0],
                               lineWidth=100,
                               lifeTime=100
                               )    
            '''                         
        for ii in range(0,15):
            distance = closest_pts[ii][8]
            ptA = closest_pts[ii][5]
            ptB = closest_pts[ii][6]
            p.addUserDebugLine(lineFromXYZ=ptA,
                               lineToXYZ=ptB,
                               lineColorRGB=[1,0,0],
                               lineWidth=1,
                               lifeTime=0,
                               replaceItemUniqueId=self.lineId_list[ii])