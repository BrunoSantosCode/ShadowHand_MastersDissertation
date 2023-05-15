import numpy as np
import scipy.optimize as opt
# for Forward Kinematics
import numpy as np
from math import cos
from math import sin
from math import radians
from math import pi as PI

E1 = 0.06       # Threshold for pinching position principal vectors (m)
E2 = 0.04       # Threshold for pinching position secundary vectors (m)
BETA = 1.0      # Scale factor
ETA1 = 0.001    # Closes distance finger/thumb (m)
ETA2 = 0.03     # Forces minimum separation between primary fingers (m)
GAMMA = 0.03    # Weight on regularizing the robotic hand angles to zero (fully opened hand)

rh, s, f = [], [], []
robot_joints_prev = np.zeros(24)


# Forward Kinematics matrix
def matrixT(a, alpha, d, theta):
    T = np.array([[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                  [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                  [0,           sin(alpha),             cos(alpha),            d],
                  [0,           0,                      0,                     1]])
    return T

########################################
### SHADOW HAND - FORWARD KINEMATICS ###
########################################

# Note: assuming that joints = [FFJ1, FFJ2, FFJ3, FFJ4, 
#                               LFJ1, LFJ2, LFJ3, LFJ4, LFJ5, 
#                               MFJ1, MFJ2, MFJ3, MFJ4,
#                               RFJ1, RFJ2, RFJ3, RFJ4,
#                               THJ1, THJ2, THJ3, THJ4, THJ5,
#                               WRJ1, WRJ2]

def getThumbTip(joints):
    f1, f2, f3, f4, f5 = joints[17], joints[18], joints[19], joints[20], joints[21]
    x0 = (16*cos(f5)*sin(f2))/5 + (19*sin(f4)*sin(f5))/5 + (11*cos(f1)*cos(f5)*sin(f2))/4 + (11*cos(f2)*cos(f5)*sin(f1))/4 + (16*cos(f2)*cos(f3)*sin(f4)*sin(f5))/5 + (16*cos(f2)*cos(f4)*sin(f3)*sin(f5))/5 + (11*cos(f1)*cos(f2)*cos(f3)*sin(f4)*sin(f5))/4 + (11*cos(f1)*cos(f2)*cos(f4)*sin(f3)*sin(f5))/4 - (11*cos(f3)*sin(f1)*sin(f2)*sin(f4)*sin(f5))/4 - (11*cos(f4)*sin(f1)*sin(f2)*sin(f3)*sin(f5))/4
    y0 = (16*sin(f2)*sin(f5))/5 - (19*cos(f5)*sin(f4))/5 + (11*cos(f1)*sin(f2)*sin(f5))/4 + (11*cos(f2)*sin(f1)*sin(f5))/4 - (16*cos(f2)*cos(f3)*cos(f5)*sin(f4))/5 - (16*cos(f2)*cos(f4)*cos(f5)*sin(f3))/5 - (11*cos(f1)*cos(f2)*cos(f3)*cos(f5)*sin(f4))/4 - (11*cos(f1)*cos(f2)*cos(f4)*cos(f5)*sin(f3))/4 + (11*cos(f3)*cos(f5)*sin(f1)*sin(f2)*sin(f4))/4 + (11*cos(f4)*cos(f5)*sin(f1)*sin(f2)*sin(f3))/4
    z0 = - (11*cos(f1 + f2 - f3 - f4))/8 - (8*cos(f3 - f2 + f4))/5 - (11*cos(f1 + f2 + f3 + f4))/8 - (19*cos(f4))/5 - (8*cos(f2 + f3 + f4))/5
    x = x0*cos(PI/4) - z0*sin(PI/4)
    y = x0*sin(PI/4) + z0*cos(PI/4)
    z = -y0
    x = x + 2.9
    y = y - 0.1
    z = z + 0.85
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

def getForeFingerTip(joints):
    f1, f2, f3, f4 = joints[0], joints[1], joints[2], joints[3]
    x = (cos(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    y = (sin(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    z = (13*sin(f1 + f2 + f3))/5 + (5*sin(f2 + f3))/2 + (9*sin(f3))/2
    x = x + 9.5
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

def getMiddleFingerTip(joints):
    f1, f2, f3, f4 = joints[9], joints[10], joints[11], joints[12]
    x = (cos(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    y = (sin(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    z = (13*sin(f1 + f2 + f3))/5 + (5*sin(f2 + f3))/2 + (9*sin(f3))/2
    x = x + 9.9
    y = y + 2.2
    return np.array([x*0.01, y*0.01, z*0.01])

def getRingFingerTip(joints):
    f1, f2, f3, f4 = joints[13], joints[14], joints[15], joints[16]
    x = (cos(-f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    y = (sin(-f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    z = (13*sin(f1 + f2 + f3))/5 + (5*sin(f2 + f3))/2 + (9*sin(f3))/2
    x = x + 9.5
    y = y + 2.2 + 2.2
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

def getLittleFingerTip(joints):
    f1, f2, f3, f4, f5 = joints[4], joints[5], joints[6], joints[7], joints[8]
    # Define the DH parameters
    a = [0, 8.66-2.071, 0, 4.5, 2.5, 2.6] # link length
    alpha = [PI/2, 0, -PI/2, 0, 0, 0] # link twist
    d = [0, 0, 0, 0, 0, 0] # link offset
    theta = [f5, radians(55), f4, f3, f2, f1] # joint angle
    # Compute the transformation matrices
    T01 = matrixT(a[0], alpha[0], d[0], theta[0])
    T12 = matrixT(a[1], alpha[1], d[1], theta[1])
    T23 = matrixT(a[2], alpha[2], d[2], theta[2])
    T34 = matrixT(a[3], alpha[3], d[3], theta[3])
    T45 = matrixT(a[4], alpha[4], d[4], theta[4])
    T56 = matrixT(a[5], alpha[5], d[5], theta[5])
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    p = T06[0:3, 3]
    x0, y0, z0 = p[0], p[1], p[2]
    x = x0*cos(radians(55)) + z0*sin(radians(55))
    y = x0*sin(radians(55)) - z0*cos(radians(55))
    z = y0
    x = x + 2.071
    y = y + 2.2 + 2.2 + 2.2
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

###############################################
### END OF SHADOW HAND - FORWARD KINEMATICS ###
###############################################


# s(di)  [Switching weight function]
def switch_weight(dist, id):
    if (dist > E1) and (id < 4):
        return 1
    elif (dist > E2) and (id >= 4):
        return 1
    elif (dist <= E1) and (id < 4):
        return 200
    elif (dist <= E2) and (id >= 4):
        return 400
    
# f(di)  [Distancing function]
def dist_function(dist, id):
    if (dist > E1) and (id < 4):
        return BETA*dist
    elif (dist > E2) and (id >= 4):
        return BETA*dist  
    elif (dist <= E1) and (id < 4):
        return ETA1
    elif (dist <= E2) and (id >= 4):
        return ETA2

# rî(qh)  [Human vectors from joints angles]
def vectors_human(th, ff, mf, rf, lf, rot, id):
    if id == 1:
        v01 = np.subtract(th, ff)
        return np.dot(rot, v01.T)
    elif id == 2:
        v02 = np.subtract(th, mf)
        return np.dot(rot, v02.T)
    elif id == 3:
        v03 = np.subtract(th, rf)
        return np.dot(rot, v03.T)
    elif id == 4:
        v04 = np.subtract(th, lf)
        return np.dot(rot, v04.T)
    elif id == 5:
        v05 = np.subtract(ff, mf)
        return np.dot(rot, v05.T)
    elif id == 6:
        v06 = np.subtract(ff, rf)
        return np.dot(rot, v06.T)
    elif id == 7:
        v07 = np.subtract(ff, lf)
        return np.dot(rot, v07.T)
    elif id == 8:
        v08 = np.subtract(rf, mf)
        return np.dot(rot, v08.T)
    elif id == 9:
        v09 = np.subtract(mf, lf)
        return np.dot(rot, v09.T)
    elif id == 10:
        v10 = np.subtract(rf, lf)
        return np.dot(rot, v10.T)
    else:
        print('ERROR: Invalid ID for hand vector!')
        exit(-1)


# ri(qr)
def vectors_robot(posTH, posFF, posMF, posRF, posLF, id):
    if id == 1:
        return posTH-posFF
    elif id == 2:
        return posTH-posMF
    elif id == 3:
        return posTH-posRF
    elif id == 4:
        return posTH-posLF
    elif id == 5:
        return posFF-posMF
    elif id == 6:
        return posFF-posRF
    elif id == 7:
        return posFF-posLF
    elif id == 8:
        return posRF-posMF
    elif id == 9:
        return posFF-posRF
    elif id == 10:
        return posRF-posLF
    else:
        print('ERROR: Invalid ID for robot vector!')
        exit(-1)

# Function to be optimized
def cost_function(robot_joints):
    posTH = getThumbTip(robot_joints)
    posFF = getForeFingerTip(robot_joints)
    posMF = getMiddleFingerTip(robot_joints)
    posRF = getRingFingerTip(robot_joints)
    posLF = getLittleFingerTip(robot_joints)
    sum = 0
    # C(qh,qa) = 0.5 * SUM (0<i<N) [ s(di) * ||ri(qa)-f(di)r^i(qh)||² + GAMMA*||qa||² ]
    for i in range(0,10):
        r_r = vectors_robot(posTH, posFF, posMF, posRF, posLF, i+1)
        sum += s[i] * (np.linalg.norm(r_r-np.multiply(f[i],(r_h[i]/np.linalg.norm(r_h[i]))))**2)
        sum += GAMMA * (np.linalg.norm(robot_joints)**2)
    cost = 0.5 * sum
    return cost


# Define bounds (degrees)
# Note: assuming that joints = [FFJ1, FFJ2, FFJ3, FFJ4, 
#                               LFJ1, LFJ2, LFJ3, LFJ4, LFJ5, 
#                               MFJ1, MFJ2, MFJ3, MFJ4
#                               RFJ1, RFJ2, RFJ3, RFJ4
#                               THJ1, THJ2, THJ3, THJ4, THJ5]

bounds_deg = ((0,90), (0,90), (-15,90), (-20,20),
              (0,90), (0,90), (-15,90), (-20,20), (0,45),
              (0,90), (0,90), (-15,90), (-20,20),
              (0,90), (0,90), (-15,90), (-20,20),
              (-15,90), (-30,30), (-12,12), (0,70), (-60,60),
              (-40,28), (-28,8))

bounds_rad = np.radians(bounds_deg)

def dexPilot_joints(hand_kp):
    global r_h, s, f
    global robot_joints_prev
    # Set initial parameters  (Robot joints = 0)
    robot_joints_0 = robot_joints_prev
    # Get relevant human hand positions
    aux0 = np.array([hand_kp[0].point.x, hand_kp[0].point.y, hand_kp[0].point.z])
    aux5 = np.array([hand_kp[5].point.x, hand_kp[5].point.y, hand_kp[5].point.z])
    aux9 = np.array([hand_kp[9].point.x, hand_kp[9].point.y, hand_kp[9].point.z])
    aux13 = np.array([hand_kp[13].point.x, hand_kp[13].point.y, hand_kp[13].point.z])
    th = np.array([hand_kp[4].point.x, hand_kp[4].point.y, hand_kp[4].point.z])
    ff = np.array([hand_kp[8].point.x, hand_kp[8].point.y, hand_kp[8].point.z])
    mf = np.array([hand_kp[12].point.x, hand_kp[12].point.y, hand_kp[12].point.z])
    rf = np.array([hand_kp[16].point.x, hand_kp[16].point.y, hand_kp[16].point.z])
    lf = np.array([hand_kp[20].point.x, hand_kp[20].point.y, hand_kp[20].point.z])
    # Rotation matrix
    v05 = np.subtract(aux5, aux0)
    v09 = np.subtract(aux9, aux0)
    v013 = np.subtract(aux13, aux0)
    z = np.cross(v05, v013)
    z = z / np.linalg.norm(z)
    y = np.cross(z, v09)
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    x = x[np.newaxis]
    y = y[np.newaxis]
    z = z[np.newaxis]
    rot = np.concatenate((x.T, y.T, z.T), axis=1).T
    # Human Vectors [array]
    r_h = [vectors_human(th, ff, mf, rf, lf, rot, i+1) for i in range(10)]
    if False:
        print('1:',r_h[0])
        print('2:',r_h[1])
        print('3:',r_h[2])
        print('4:',r_h[3])
    # Distances [array]
    d = [np.linalg.norm(r_h[i]) for i in range(10)]
    if True:
        print('1:',d[0])
        print('2:',d[1])
        print('3:',d[2])
        print('4:',d[3])
    # Switching weights [array]
    s = [switch_weight(d[i], i) for i in range(10)]
    # Distancing [array]
    f = [dist_function(d[i], i) for i in range(10)]
    result = opt.minimize(cost_function, robot_joints_0, method='SLSQP', bounds=bounds_rad)
    robot_joints_prev = result.x
    return result.x, result.fun