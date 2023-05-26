#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
def talker():
    pub2 = rospy.Publisher('shadow_joints', Float64MultiArray, queue_size=1)
    rospy.init_node('send_shadow_joints')
    open = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
            'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
            'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
            'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
            'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
            'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
    close = {'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
            'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
            'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
            'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
            'rh_THJ1': 0.52, 'rh_THJ2': 0.61, 'rh_THJ3': 0.0, 'rh_THJ4': 1.20, 'rh_THJ5': 0.17,
            'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
    grasp1 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 1.0, 'rh_FFJ3': 0.7858461013048577, 'rh_FFJ4': 0.0,
              'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
              'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
              'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
              'rh_THJ1': -1.256014797634819e-05, 'rh_THJ2': 0.05002753347458544, 'rh_THJ3': 0.023103459764708845, 'rh_THJ4': 1.036535158324675, 'rh_THJ5': 0.5304669523651526,
              'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
    grasp2 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 1.282407938672396, 'rh_FFJ3': 0.6206287904270582, 'rh_FFJ4': -0.11128212790587894,
              'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
              'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
              'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
              'rh_THJ1': -0.007974038489365468, 'rh_THJ2': 0.4257676647231374, 'rh_THJ3': -0.008827405515457718, 'rh_THJ4': 1.0602377228405477, 'rh_THJ5': 0.4064127320619911,
              'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
    grasp3_1 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.6855010285793518, 'rh_FFJ3': 0.8778654389615375, 'rh_FFJ4': -0.11234783711794277,
                'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                'rh_THJ1': 0.04780693252039506, 'rh_THJ2': 0.39167183008648965, 'rh_THJ3': -0.010286095762729664, 'rh_THJ4': 1.102183407997245, 'rh_THJ5': 0.3262467647139002,
                'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
    grasp3_2 = {'rh_FFJ1': 0.0, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 0.8778654389615375, 'rh_FFJ4': -0.11234783711794277,
                'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                'rh_THJ1': 0.04780693252039506, 'rh_THJ2': 0.39167183008648965, 'rh_THJ3': -0.010286095762729664, 'rh_THJ4': 1.102183407997245, 'rh_THJ5': 0.3262467647139002,
                'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
    grasp4_1 = {'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 1.3109351390380346, 'rh_MFJ4': 0.349,
                'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                'rh_THJ1': 0.6332039731068623, 'rh_THJ2': 0.43818536870966607, 'rh_THJ3': 0.2060327901731422, 'rh_THJ4': 1.221, 'rh_THJ5': 0.22480603191738607,
                'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
    grasp4_2 = {'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                'rh_MFJ1': 0.0, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.3109351390380346, 'rh_MFJ4': 0.349,
                'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                'rh_THJ1': 0.6332039731068623, 'rh_THJ2': 0.43818536870966607, 'rh_THJ3': 0.2060327901731422, 'rh_THJ4': 1.221, 'rh_THJ5': 0.22480603191738607,
                'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
    #{'rh_FFJ1': 0.013572548906637949, 'rh_FFJ2': 0.5063751316641443, 'rh_FFJ3': 1.0417903240533037, 'rh_FFJ4': 0.3380985879834396, 'rh_LFJ1': 1.3337952571905245, 'rh_LFJ2': 1.683722746661628, 'rh_LFJ3': 1.5503962015788462, 'rh_LFJ4': -0.0581670083354725, 'rh_LFJ5': 0.02635596439947014, 'rh_MFJ1': 0.004336554276427651, 'rh_MFJ2': 0.5160003764801178, 'rh_MFJ3': 1.2085022172365107, 'rh_MFJ4': 0.33416019339832687, 'rh_RFJ1': 1.4925725660541045, 'rh_RFJ2': 1.6478381084905591, 'rh_RFJ3': 1.5528115013509025, 'rh_RFJ4': 0.006761265642872095, 'rh_THJ1': 0.639308360882898, 'rh_THJ2': 0.12946800318880675, 'rh_THJ3': 0.2075504576243693, 'rh_THJ4': 1.1299958970929687, 'rh_THJ5': 0.42619418660431235, 'rh_WRJ1': -0.6872155539917254, 'rh_WRJ2': -0.01041616141776023}
    # Joints order
    # rh_WRJ2  rh_WRJ1
    # rh_FFJ4  rh_FFJ3  rh_FFJ2  rh_FFJ1
    # rh_LFJ5  rh_LFJ4  rh_LFJ3  rh_LFJ1
    # rh_MFJ4  rh_MFJ3  rh_MFJ2  rh_MFJ1
    # rh_RFJ4  rh_RFJ3  rh_RFJ2  rh_RFJ1
    # rh_THJ5  rh_THJ4  rh_THJ3  rh_THJ2  rh_THJ1
    goal = Float64MultiArray()
    ### SET JOINT ANGLES HERE ###
    set_joints = grasp4_1
    goal.data.append(set_joints['rh_WRJ2'])
    goal.data.append(set_joints['rh_WRJ1'])
    goal.data.append(set_joints['rh_FFJ4'])
    goal.data.append(set_joints['rh_FFJ3'])
    goal.data.append(set_joints['rh_FFJ2'])
    goal.data.append(set_joints['rh_FFJ1'])
    goal.data.append(set_joints['rh_LFJ5'])
    goal.data.append(set_joints['rh_LFJ4'])
    goal.data.append(set_joints['rh_LFJ3'])
    goal.data.append(set_joints['rh_LFJ2'])
    goal.data.append(set_joints['rh_LFJ1'])
    goal.data.append(set_joints['rh_MFJ4'])
    goal.data.append(set_joints['rh_MFJ3'])
    goal.data.append(set_joints['rh_MFJ2'])
    goal.data.append(set_joints['rh_MFJ1'])
    goal.data.append(set_joints['rh_RFJ4'])
    goal.data.append(set_joints['rh_RFJ3'])
    goal.data.append(set_joints['rh_RFJ2'])
    goal.data.append(set_joints['rh_RFJ1'])
    goal.data.append(set_joints['rh_THJ5'])
    goal.data.append(set_joints['rh_THJ4'])
    goal.data.append(set_joints['rh_THJ3'])
    goal.data.append(set_joints['rh_THJ2'])
    goal.data.append(set_joints['rh_THJ1'])
    pub2.publish(goal)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass





