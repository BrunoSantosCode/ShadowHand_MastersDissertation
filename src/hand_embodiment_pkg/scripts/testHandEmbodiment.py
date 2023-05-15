#!/usr/bin/env python3

#* * * * * * * * * * test0.py * * * * * * * * * * *#
#*   Tests hand_embodiment                        *#
#*   Inputs: hardcoded hand keypoints positions   *#
#*   Outputs: Shadow Hand joint angles            *#
#* * * ** * * * * * * * * * * * * * * * * * * * * *#

import imp
import time
import rospkg
import numpy as np
#import pytransform3d.transformations as pt
from hand_embodiment_pkg.hand_embodiment.pipelines import MoCapToRobot


PACKAGE_PATH = rospkg.RosPack().get_path('hand_embodiment_pkg')

MANO_CONFIG_PATH = PACKAGE_PATH + "/examples/config/mano/20210616_april.yaml"
FINGERS = ["thumb", "index", "middle", "ring", "little"]

# Create a MoCapToRobot object for the Shadow hand
shadow_hand = MoCapToRobot(
    hand = "shadow",
    mano_config = MANO_CONFIG_PATH,
    use_fingers = FINGERS,
    measure_time = True
)

# Test with frame 2000 of "test/data/recording.tsv"
								
line2000 = [197.090, 186.785, 355.568,  # hand_top          [0:3]
            148.571, 212.874, 351.756,  # hand_left         [3:6]
            152.791, 181.826, 360.058,  # hand_right        [6:6]
            187.023, 258.098, 238.769,  # thumb_tip         [9:12]
            173.292, 248.519, 266.282,  # thumb_middle      [12:15]
            286.472, 143.090, 312.203,  # little_tip        [15:18]
            251.295, 140.943, 328.034,  # little_middle     [18:21] 
            318.117, 167.697, 311.492,  # ring_tip          [21:24]
            264.293, 164.880, 337.638,  # ring_middle       [24:27]
            266.009, 204.709, 340.339,  # middle_middle     [27:30]
            327.375, 195.134, 316.995,  # middle_tip        [30:33]
            312.663, 236.605, 322.433,  # index_tip         [33:36]
	        255.355, 231.975, 342.499,  # index_middle      [36:39] 
           ]   

# Define the positions of the hand markers (in millimeters)
hand_markers = [np.array(line2000[0:3]),   # Hand top marker
                np.array(line2000[3:6]),   # Hand left marker
                np.array(line2000[6:9]),   # Hand right marker
]

# Define the positions of the finger markers (in millimeters)
finger_markers = {
    "thumb" : np.array(line2000[9:12]),
    "index" : np.array(line2000[33:36]),
    "middle": np.array(line2000[30:33]),
    "ring"  : np.array(line2000[21:24]),
    "little": np.array(line2000[15:18])
}

start_time = time.time()

# Estimate the joint angles of the Shadow hand from the marker positions
_, joint_angles = shadow_hand.estimate(hand_markers, finger_markers)

end_time = time.time()

# Print elapsed time
print(f"Elapsed time: {end_time-start_time} seconds")

# Print the joint angles
print("Joint angles:")
print(joint_angles)

