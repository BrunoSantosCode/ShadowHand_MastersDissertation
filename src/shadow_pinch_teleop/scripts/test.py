import tf
import numpy as np

# Clip frame position and orientation
clip_position = np.array([0.8090350904948178, -0.05905678621987004, 0.000595339747434958])
clip_orientation = np.array([-0.7419143595894909, -0.6699794054594519, 0.025714768389053667, -0.005424940719662695])

# UR5 frame position and orientation
ur5_position = np.array([0.5569875068079165, -0.21011642882890044, 0.24878966345710363])
ur5_orientation = np.array([-0.9293259401298217, -0.2110456508124238, 0.284001015328393, -0.10562411451664502])

# Create transformation matrices
clip_transformation = tf.TransformerROS().fromTranslationRotation(clip_position, clip_orientation)
ur5_transformation = tf.TransformerROS().fromTranslationRotation(ur5_position, ur5_orientation)

# Calculate the transformation from UR5 to clip frame
ur5_to_clip_transformation = np.linalg.inv(clip_transformation) @ ur5_transformation

# Extract the position and orientation of UR5 in the clip frame
ur5_position_in_clip = ur5_to_clip_transformation[:3, 3]
ur5_orientation_in_clip = tf.transformations.quaternion_from_matrix(ur5_to_clip_transformation)

# Print the UR5 end-effector coordinates in the clip frame
print("UR5 end-effector coordinates in the clip frame:")
print("Position:", ur5_position_in_clip)
print("Orientation:", ur5_orientation_in_clip)
