import numpy as np

AVERAGE_N = 5

joints = np.zeros((24, AVERAGE_N))

joints[0][0] = 1
joints[0][1] = 2
joints[0][2] = 3
joints[0][3] = 4
joints[0][4] = 5
joints[1][0] = 1
joints[1][1] = 2
joints[1][2] = 3
joints[1][3] = 4
joints[1][4] = 5

joints[0] = np.roll(joints[0], -1)

joints[0][-1] = 6

print(np.mean(joints[1]))

a = np.array([[1,2,3]]).T
print(a[2][0])