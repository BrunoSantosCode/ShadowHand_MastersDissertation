import numpy as np
import scipy.optimize as opt
# for Forward Kinematics
import numpy as np
from math import cos
from math import sin
from math import radians
from math import pi as PI

# Algorithm parameters
BETA = 1.0      # Scale factor


# Global vars
th, ff, mf, rf, lf = 0, 0, 0, 0, 0
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

WRIST_X = 3.4
WRIST_Y = 3.4

def getThumbTip(joints):
    f1, f2, f3, f4, f5 = joints[17], joints[18], joints[19], joints[20], joints[21]
    x0 = (16*cos(f5)*sin(f2))/5 + (19*sin(f4)*sin(f5))/5 + (11*cos(f1)*cos(f5)*sin(f2))/4 + (11*cos(f2)*cos(f5)*sin(f1))/4 + (16*cos(f2)*cos(f3)*sin(f4)*sin(f5))/5 + (16*cos(f2)*cos(f4)*sin(f3)*sin(f5))/5 + (11*cos(f1)*cos(f2)*cos(f3)*sin(f4)*sin(f5))/4 + (11*cos(f1)*cos(f2)*cos(f4)*sin(f3)*sin(f5))/4 - (11*cos(f3)*sin(f1)*sin(f2)*sin(f4)*sin(f5))/4 - (11*cos(f4)*sin(f1)*sin(f2)*sin(f3)*sin(f5))/4
    y0 = (16*sin(f2)*sin(f5))/5 - (19*cos(f5)*sin(f4))/5 + (11*cos(f1)*sin(f2)*sin(f5))/4 + (11*cos(f2)*sin(f1)*sin(f5))/4 - (16*cos(f2)*cos(f3)*cos(f5)*sin(f4))/5 - (16*cos(f2)*cos(f4)*cos(f5)*sin(f3))/5 - (11*cos(f1)*cos(f2)*cos(f3)*cos(f5)*sin(f4))/4 - (11*cos(f1)*cos(f2)*cos(f4)*cos(f5)*sin(f3))/4 + (11*cos(f3)*cos(f5)*sin(f1)*sin(f2)*sin(f4))/4 + (11*cos(f4)*cos(f5)*sin(f1)*sin(f2)*sin(f3))/4
    z0 = - (11*cos(f1 + f2 - f3 - f4))/8 - (8*cos(f3 - f2 + f4))/5 - (11*cos(f1 + f2 + f3 + f4))/8 - (19*cos(f4))/5 - (8*cos(f2 + f3 + f4))/5
    x = x0*cos(PI/4) - z0*sin(PI/4)
    y = x0*sin(PI/4) + z0*cos(PI/4)
    z = -y0
    x = x + 2.9 + WRIST_X
    y = y - 0.1 - WRIST_Y
    z = z + 0.85
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

def getForeFingerTip(joints):
    f1, f2, f3, f4 = joints[0], joints[1], joints[2], joints[3]
    x = (cos(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    y = (sin(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    z = (13*sin(f1 + f2 + f3))/5 + (5*sin(f2 + f3))/2 + (9*sin(f3))/2
    x = x + 9.5 + WRIST_X
    y = y - WRIST_Y
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

def getMiddleFingerTip(joints):
    f1, f2, f3, f4 = joints[9], joints[10], joints[11], joints[12]
    x = (cos(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    y = (sin(f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    z = (13*sin(f1 + f2 + f3))/5 + (5*sin(f2 + f3))/2 + (9*sin(f3))/2
    x = x + 9.9 + WRIST_X
    y = y + 2.2 - WRIST_Y
    return np.array([x*0.01, y*0.01, z*0.01])

def getRingFingerTip(joints):
    f1, f2, f3, f4 = joints[13], joints[14], joints[15], joints[16]
    x = (cos(-f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    y = (sin(-f4)*(25*cos(f2 + f3) + 45*cos(f3) + 26*cos(f1 + f2 + f3)))/10
    z = (13*sin(f1 + f2 + f3))/5 + (5*sin(f2 + f3))/2 + (9*sin(f3))/2
    x = x + 9.5 + WRIST_X
    y = y + 2.2 + 2.2 - WRIST_Y
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
    x = x + 2.071 + WRIST_X
    y = y + 2.2 + 2.2 + 2.2 - WRIST_Y
    return np.array([x*0.01, y*0.01, z*0.01])   # [cm -> m]

###############################################
### END OF SHADOW HAND - FORWARD KINEMATICS ###
###############################################


# Function to be optimized
def cost_function(robot_joints):
    global th, ff, mf, rf, lf
    posTH = getThumbTip(robot_joints)
    posFF = getForeFingerTip(robot_joints)
    posMF = getMiddleFingerTip(robot_joints)
    posRF = getRingFingerTip(robot_joints)
    posLF = getLittleFingerTip(robot_joints)
    
    cost = np.linalg.norm(np.subtract(posTH,th)) + \
           np.linalg.norm(np.subtract(posFF,ff)) + \
           np.linalg.norm(np.subtract(posMF,mf)) + \
           np.linalg.norm(np.subtract(posRF,rf)) + \
           np.linalg.norm(np.subtract(posLF,lf)) 

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
    global robot_joints_prev, th, ff, mf, rf, lf
    # Set initial parameters  (Robot joints = 0)
    robot_joints_0 = robot_joints_prev
    # Fingertips
    wr = np.array([hand_kp[0].point.x, hand_kp[0].point.y, hand_kp[0].point.z])
    th = np.array([hand_kp[4].point.x, hand_kp[4].point.y, hand_kp[4].point.z])
    ff = np.array([hand_kp[8].point.x, hand_kp[8].point.y, hand_kp[8].point.z])
    mf = np.array([hand_kp[12].point.x, hand_kp[12].point.y, hand_kp[12].point.z])
    rf = np.array([hand_kp[16].point.x, hand_kp[16].point.y, hand_kp[16].point.z])
    lf = np.array([hand_kp[20].point.x, hand_kp[20].point.y, hand_kp[20].point.z])
    # Rotation matrix
    aux0 = np.array([hand_kp[0].point.x, hand_kp[0].point.y, hand_kp[0].point.z])
    aux5 = np.array([hand_kp[5].point.x, hand_kp[5].point.y, hand_kp[5].point.z])
    aux9 = np.array([hand_kp[9].point.x, hand_kp[9].point.y, hand_kp[9].point.z])
    aux13 = np.array([hand_kp[13].point.x, hand_kp[13].point.y, hand_kp[13].point.z])
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
    # Re-oriented hand fingertips
    wr = np.dot(rot, np.array(wr).T)
    th = np.dot(rot, np.array(th).T)
    th = np.subtract(th, wr)
    ff = np.dot(rot, np.array(ff).T)
    ff = np.subtract(ff, wr)
    mf = np.dot(rot, np.array(mf).T)
    mf = np.subtract(mf, wr)
    rf = np.dot(rot, np.array(rf).T)
    rf = np.subtract(rf, wr)
    lf = np.dot(rot, np.array(lf).T)
    lf = np.subtract(lf, wr)

    if False:
        print('th:',th)
        print('ff:',ff)
        print('mf:',mf)
        print('rf:',rf)
        print('lf:',lf)

    
    result = opt.minimize(cost_function, robot_joints_0, method='SLSQP', bounds=bounds_rad)
    robot_joints_prev = result.x
    return result.x, result.fun