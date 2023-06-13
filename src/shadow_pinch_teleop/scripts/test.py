import numpy as np

x1 = 1.0
y1 = 2.0
z1 = 3.0

x2 = 4.0
y2 = 5.0
z2 = 6.0

# Define the coordinates of points A and B
A = np.array([1.0, 2.0, 3.0])  # Coordinates of point A
B = np.array([4.0, 5.0, 6.0])  # Coordinates of point B
print(A)
# Calculate the translation vector between A and B
translation = B - A

# Calculate the distance between A and B
distance = np.linalg.norm(translation)

# Normalize the translation vector
normalized_translation = translation / distance

# Calculate the rotation axis (cross product of A and B)
rotation_axis = np.cross(A, B)
rotation_axis /= np.linalg.norm(rotation_axis)

# Calculate the rotation angle using the dot product and arccosine
dot_product = np.dot(A, B)
rotation_angle = np.arccos(dot_product / (np.linalg.norm(A) * np.linalg.norm(B)))

# Create the rotation matrix using the Rodrigues' formula
rotation_matrix = np.eye(3) + np.sin(rotation_angle) * np.cross(np.eye(3), rotation_axis) + (1 - np.cos(rotation_angle)) * np.dot(np.outer(rotation_axis, rotation_axis), np.eye(3))

# Create the homogeneous transformation matrix
homogeneous_matrix = np.eye(4)
homogeneous_matrix[:3, :3] = rotation_matrix
homogeneous_matrix[:3, 3] = normalized_translation

# Transform point A to point B using the homogeneous matrix
transformed_A = np.dot(homogeneous_matrix, np.append(A, 1))[:3]

# Print the transformed coordinates of A
print("Transformed A:", transformed_A)