import numpy as np
from modern_robotics import *
def w_p_to_slit(w,p):

    Slist =[];

    for i in range(0,len(w)):

        v = -np.cross(np.array(w[i]),np.array(p[i]))

        wi =np.array(w[i])

        S = [wi[0],wi[1],wi[2],v[0],v[1],v[2]];

        Slist.append(S)

    return np.array(Slist).T

H1 = 0.3;
H2 = 0.45;
H3 = 0.350;
H4 = 0.228;
W1 = 0.0035;
W2 = 0.183;        
w =[]

p_ =[]

#right arm
base_p = []
base_w = []
base_p.append([0,0,0])
base_p.append([0,0,H1])
base_p.append([0,0,H1+H2])
base_p.append([0,-W1,H1+H2+H3])
base_p.append([0,-W1,H1+H2+H3])
base_p.append([0,-W1-W2,H1+H2+H3])

base_w.append([0 ,0,  1]);
base_w.append([0 ,-1, 0]);
base_w.append([0 ,-1,  0]);
base_w.append([0 ,0, 1]);
base_w.append([0 ,-1, 0]);
base_w.append([0  ,0, 1]);
M = np.array([[1,0,0,0],[0 ,1 ,0 ,-W1-W2 ],[0 ,0 ,1 ,H1+H2+H3+H4],[0 ,0 ,0 ,1 ]])
Slist = w_p_to_slit(base_w,base_p);
Blist = Adjoint(TransInv(M))@Slist;
M = np.array([[1,0,0,0],[0 ,1 ,0 ,-W1-W2 ],[0 ,0 ,1 ,H1+H2+H3+H4],[0 ,0 ,0 ,1 ]])
print(Slist)
print(Blist)
print(M)
