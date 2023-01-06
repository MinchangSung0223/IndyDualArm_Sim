import bpy
import numpy as np
from math import sqrt,cos,sin
from modern_robotics import TransInv,Adjoint
import json
def setEnv(linkHide = True,linkCenterHide = True,linkTcpHide = True,jointHide =False,meshHide = False):
    link_name_list = []
    joint_name_list = []
    mesh_name_list = []
    for i in range(0,7):
        r_link_name = "r_"+str(i);
        r_mesh_name = "r_Indy7_"+str(i);
        r_link_center_name = "r_"+str(i)+"_center";
        l_link_name = "l_"+str(i);    
        l_mesh_name = "l_Indy7_"+str(i);
        l_link_center_name = "l_"+str(i)+"_center";    
        link_name_list.append(r_link_name);
        link_name_list.append(l_link_name);    
        bpy.data.objects[r_link_name].hide_set(linkHide)
        bpy.data.objects[l_link_name].hide_set(linkHide)    
        bpy.data.objects[r_link_center_name].hide_set(linkCenterHide)
        bpy.data.objects[l_link_center_name].hide_set(linkCenterHide)    
        bpy.data.objects['r_tcp'].hide_set(linkHide)    
        bpy.data.objects['l_tcp'].hide_set(linkHide)  
        bpy.data.objects['r_tcp_'].hide_set(linkTcpHide)    
        bpy.data.objects['l_tcp_'].hide_set(linkTcpHide)      
        bpy.data.objects['world'].hide_set(linkHide)  
        bpy.data.objects['body'].hide_set(linkHide)  
        bpy.data.objects[r_mesh_name].hide_set(meshHide)
        bpy.data.objects[l_mesh_name].hide_set(meshHide)    
        bpy.data.objects['body_mesh'].hide_set(meshHide)  
        bpy.data.objects['table'].hide_set(meshHide)      
        bpy.data.objects['table'].hide_set(meshHide)        
        bpy.data.objects['camera'].hide_set(meshHide)          
        bpy.data.objects['camera1'].hide_set(meshHide)          
        bpy.data.objects['camera2'].hide_set(meshHide)          
        bpy.data.objects['head'].hide_set(meshHide)                  
        bpy.data.objects['Plane'].hide_set(meshHide)           
        if (i+1<7):
            r_joint_name = "r_joint_"+str(i+1)
            l_joint_name = "l_joint_"+str(i+1)
            bpy.data.objects[r_joint_name].hide_set(jointHide) 
            bpy.data.objects[l_joint_name].hide_set(jointHide)         
            joint_name_list.append(r_joint_name);
            joint_name_list.append(l_joint_name); 
    print(joint_name_list)