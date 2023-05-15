import numpy as np

# 3D Hand Keypoints
wrist0   = np.array([3.0, 1.0, 0.0])
thumb1   = np.array([4.0, 2.0, 0.0])
thumb2   = np.array([5.0, 3.0, 0.0])
thumb3   = np.array([5.0, 4.0, 0.0])
thumb4   = np.array([5.0, 5.0, 0.0])
index5   = np.array([4.0, 4.0, 0.0])
index6   = np.array([4.0, 5.0, 0.0])
index7   = np.array([4.0, 6.0, 0.0])
index8   = np.array([4.0, 7.0, 0.0])
middle9  = np.array([3.0, 4.0, 0.0])
middle10 = np.array([3.0, 5.0, 0.0])
middle11 = np.array([3.0, 6.0, 0.0])
middle12 = np.array([3.0, 7.0, 0.0])
ring13   = np.array([2.0, 4.0, 0.0])
ring14   = np.array([2.0, 5.0, 0.0])
ring15   = np.array([2.0, 6.0, 0.0])
ring16   = np.array([2.0, 7.0, 0.0])
little17 = np.array([1.0, 4.0, 0.0])
little18 = np.array([1.0, 5.0, 0.0])
little19 = np.array([1.0, 6.0, 0.0])
little20 = np.array([1.0, 7.0, 0.0])

# Get target referential axis
v05 = np.subtract(index5, wrist0)
v09 = np.subtract(middle9, wrist0)
v013 = np.subtract(ring13, wrist0)

z = np.cross(v05, v013)
z = z / np.linalg.norm(z)

y = np.cross(z, v09)
y = y / np.linalg.norm(y)

x = np.cross(y, z)

z_check = np.cross(x, y)

x = x[np.newaxis]
y = y[np.newaxis]
z = z[np.newaxis]

if False:
    print('x:', x)
    print('y:', y)
    print('z:', z)
    print('z_check:', z_check)

rot = np.concatenate((x.T, y.T, z.T), axis=1).T

if False:
    print('Rotation matrix:')
    print(rot)

v_thumb_index_orig = np.subtract(index8, thumb4)
v_thumb_index_orig = v_thumb_index_orig[np.newaxis]

v_thumb_index_new = np.dot(rot, v_thumb_index_orig.T)

if False:
    print('Original vector:',v_thumb_index_orig)
    print('New vector:',v_thumb_index_new.T)

# Ten DexPilot vectors
v01 = np.subtract(thumb4, index8)
v02 = np.subtract(thumb4, middle12)
v03 = np.subtract(thumb4, ring16)
v04 = np.subtract(thumb4, little20)
v05 = np.subtract(index8, middle12)
v06 = np.subtract(index8, ring16)
v07 = np.subtract(index8, little20)
v08 = np.subtract(ring16, middle12)
v09 = np.subtract(middle12, little20)
v10 = np.subtract(ring16, little20)

# Convert vectors orientation
v01_new = np.dot(rot, v01.T)
v02_new = np.dot(rot, v02.T)
v03_new = np.dot(rot, v03.T)
v04_new = np.dot(rot, v04.T)
v05_new = np.dot(rot, v05.T)
v06_new = np.dot(rot, v06.T)
v07_new = np.dot(rot, v07.T)
v08_new = np.dot(rot, v08.T)
v09_new = np.dot(rot, v09.T)
v10_new = np.dot(rot, v10.T)

if True:
    print('v01:', v01.T, 'v01_new:', v01_new.T)
    print('v02:', v02.T, 'v02_new:', v02_new.T)
    print('v03:', v03.T, 'v03_new:', v03_new.T)
    print('v04:', v04.T, 'v04_new:', v04_new.T)
    print('v05:', v05.T, 'v05_new:', v05_new.T)
    print('v06:', v06.T, 'v06_new:', v06_new.T)
    print('v07:', v07.T, 'v07_new:', v07_new.T)
    print('v08:', v08.T, 'v08_new:', v08_new.T)
    print('v09:', v09.T, 'v09_new:', v09_new.T)
    print('v10:', v10.T, 'v10_new:', v10_new.T)