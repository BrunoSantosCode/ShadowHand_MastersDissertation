#!/usr/bin/env python3

import rospy
import threading
import numpy as np
import matplotlib.pyplot as plt
from openpose_mujoco.msg import Frame  # 'openpose_mujoco' is the package name
                                       # Frame is inside 'msg' folder
                                       # Path: openpose_mujoco/msg/Frame.msg

first = True
x = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
y = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

# OpenPose callback
def openPose_CB(msg):
    global first, x, y
    
    # Get hand information from OpenPose
    person = msg.persons[0]
    hand_kp = person.rightHandParts
    # Get relevant human hand positions
    aux0 = np.array([hand_kp[0].point.x, hand_kp[0].point.y, hand_kp[0].point.z])
    aux5 = np.array([hand_kp[5].point.x, hand_kp[5].point.y, hand_kp[5].point.z])
    aux9 = np.array([hand_kp[9].point.x, hand_kp[9].point.y, hand_kp[9].point.z])
    aux13 = np.array([hand_kp[13].point.x, hand_kp[13].point.y, hand_kp[13].point.z])
    # Rotation matrix calculations
    v05 = np.subtract(aux5, aux0)
    v09 = np.subtract(aux9, aux0)
    v013 = np.subtract(aux13, aux0)
    z_rot = np.cross(v05, v013)
    z_rot = z_rot / np.linalg.norm(z_rot)
    y_rot = np.cross(z_rot, v09)
    y_rot = y_rot / np.linalg.norm(y_rot)
    x_rot = np.cross(y_rot, z_rot)
    if False:
        z_check = np.cross(x_rot, y_rot)
        print(z_rot)
        print(z_check)
    x_rot = x_rot[np.newaxis]
    y_rot = y_rot[np.newaxis]
    z_rot = z_rot[np.newaxis]
    # Rotation matrix
    rot = np.concatenate((x_rot.T, y_rot.T, z_rot.T), axis=1).T
    # Get original coordinates
    x_aux = [kp.point.x for kp in hand_kp]
    y_aux = [kp.point.y for kp in hand_kp]
    z_aux = [kp.point.z for kp in hand_kp]
    # Referential transform
    wrist = np.dot(rot, np.array([[x_aux[0],y_aux[0],z_aux[0]]]).T)
    for i in range(0, len(x_aux)):
        aux = np.dot(rot, np.array([[x_aux[i],y_aux[i],z_aux[i]]]).T)
        x[i] = aux[0][0] - wrist[0][0] 
        y[i] = aux[1][0] - wrist[1][0]

    

    if first:
        plot_thread = threading.Thread(target=update_plot)
        plot_thread.start()
        first = False

    

def update_plot():
    while True:
        global x, y
        # Check if all keypoints were detected
        if (len(x) != 21) or (len(y) != 21):
            print('HERE')
            continue
        plt.clf()
        for i, (xi, yi) in enumerate(zip(x, y)):
            plt.scatter(xi, yi, s=150, color='yellow', zorder=0)
            plt.text(xi, yi, i, fontsize=12, ha='center', va='center', zorder=1)
        plt.plot([x[0], x[1]], [y[0], y[1]], color='red', zorder=-1)
        plt.plot([x[1], x[2]], [y[1], y[2]], color='red', zorder=-1)
        plt.plot([x[2], x[3]], [y[2], y[3]], color='red', zorder=-1)
        plt.plot([x[3], x[4]], [y[3], y[4]], color='red', zorder=-1)
        plt.plot([x[0], x[5]], [y[0], y[5]], color='yellow', zorder=-1)
        plt.plot([x[5], x[6]], [y[5], y[6]], color='yellow', zorder=-1)
        plt.plot([x[6], x[7]], [y[6], y[7]], color='yellow', zorder=-1)
        plt.plot([x[7], x[8]], [y[7], y[8]], color='yellow', zorder=-1)
        plt.plot([x[0],  x[9]],  [y[0],  y[9]],  color='green', zorder=-1)
        plt.plot([x[9],  x[10]], [y[9],  y[10]], color='green', zorder=-1)
        plt.plot([x[10], x[11]], [y[10], y[11]], color='green', zorder=-1)
        plt.plot([x[11], x[12]], [y[11], y[12]], color='green', zorder=-1)
        plt.plot([x[0],  x[13]], [y[0], y[13]],  color='blue', zorder=-1)
        plt.plot([x[13], x[14]], [y[13], y[14]], color='blue', zorder=-1)
        plt.plot([x[14], x[15]], [y[14], y[15]], color='blue', zorder=-1)
        plt.plot([x[15], x[16]], [y[15], y[16]], color='blue', zorder=-1)
        plt.plot([x[0],  x[17]], [y[0], y[17]],  color='cyan', zorder=-1)
        plt.plot([x[17], x[18]], [y[17], y[18]], color='cyan', zorder=-1)
        plt.plot([x[18], x[19]], [y[18], y[19]], color='cyan', zorder=-1)
        plt.plot([x[19], x[20]], [y[19], y[20]], color='cyan', zorder=-1)
        plt.xlim(min(x)-0.1, max(x)+0.1)
        plt.ylim(min(y)-0.1, max(y)+0.1)
        plt.draw()
        plt.pause(0.001)
    

# Subscribes to hand joint angles from OpenPose
def openPose_sub():
    rospy.Subscriber('frame_topic', Frame, openPose_CB, queue_size=1)

    plt.ion()
    plt.show()

    rospy.spin()    


if __name__ == "__main__":
    rospy.init_node('plot_debug')
    
    print("Node is running...")  

    openPose_sub()