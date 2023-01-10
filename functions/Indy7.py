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
D2R = np.pi/180.0
class Indy7:
    """
    Indy7 class
    """
    def __init__(self,p,urdf_name = "model/indy7.urdf", CONTROL_FREQ = 240.0):
        self.active_joint_num_list = [1,2,3,4,5,6]
        self.tcp = 7
        self.MAX_TORQUES = np.array([431.97,431.97,197.23,79.79,79.79,79.79]) # unit [N]
        self.HomePos = [0.146 ,0.814, 1.308, 0.615 ,1.030 ,-0.070]
        self.p = p
        self.p.connect(self.p.GUI)
        self.p.setGravity(0, 0, -9.8)
        self.p.setRealTimeSimulation(0)
        timeStep = 1/CONTROL_FREQ;
        self.p.setTimeStep(timeStep)
        self.p.setPhysicsEngineParameter(fixedTimeStep = timeStep)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robotId = self.p.loadURDF(urdf_name, [0, 0, 0.0],[0, 0, 0, 1],flags=self.p.URDF_USE_SELF_COLLISION)
        planeId = self.p.loadURDF("plane.urdf",[0, 0, 0.001])
        
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_GUI,0)
        self.p.resetDebugVisualizerCamera(1,90,-15,[0,0,0.5])
        self.p.resetBasePositionAndOrientation(self.robotId, [0, 0, 0], [0, 0, 0, 1])
       # active_joint_list,active_joint_num_list,eef_num=self.getActiveJointList(self.robotId)
        self.p.enableJointForceTorqueSensor(self.robotId,self.tcp,True)
        ## initialize all joint
        self.initializeActiveJoints(self.robotId,self.active_joint_num_list)

        self.data={}
        self.data["t"]=[]
        self.data["q_rel"]=[]
        self.data["T_rel"]=[]
        self.data["Xd"]=[]

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

    def getJointStates(self):
        j_list = self.active_joint_num_list;
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

    def setJointStates(self,q):
        #print(self.active_joint_num_list)
        for i in range(0,len(q)):
            self.p.setJointMotorControl2(self.robotId, self.active_joint_num_list[i], self.p.POSITION_CONTROL,targetPosition=q[i], force=self.MAX_TORQUES[i])  

    def resetJointStates(self,q):
        
        for i in range(0,len(q)):

            self.p.resetJointState(self.robotId,self.active_joint_num_list[i],q[i])

    def setData(self,t,q,T,Xd,F_r,F_l):
        self.data["t"].append(t)
        self.data["q"].append(q)
        self.data["T"].append(T)
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
    def getFT(self):
        #print(p.getJointState(self.robotId,self.tcp-1))
        val = np.array(self.p.getJointState(self.robotId,self.tcp)[2]).T
        norm_val =sqrt(val[3]*val[3]+val[4]*val[4]+val[5]*val[5])
        return np.array([val[3],val[4],val[5],val[0],val[1],val[2]]).T

    def getActuatedTorques(self):
        actuated_torques=[]
        for i in range(len(self.active_joint_num_list)):
            tempState = self.p.getJointState(self.robotId,self.active_joint_num_list[i])
            actuated_torques.append(tempState[3])
        return np.array(actuated_torques)

    def getEEFPose(self):
        Teef = np.eye(4);
        ret = p.getLinkState(self.robotId,self.tcp,1,1)
        pos = ret[0]
        orn = ret[1]    
        Teef[0:3,3] = np.array(pos).T
        Teef[0:3,0:3] = np.reshape(self.p.getMatrixFromQuaternion(orn),(3,3))
        return Teef 
    def step(self)      :
        #self.t += self.timeStep
        self.p.stepSimulation()
    def setExternalForce(self,force):
        T = self.getEEFPose()
        self.p.applyExternalForce(self.robotId,self.tcp,force,[0,0,0],self.p.LINK_FRAME)