import numpy as np

A = np.array([[1.0, 0.1],
              [0.0, 1.0]])
H = np.eye(2)
Q = np.diag([1e-4, 1e-4])
R = np.diag([1e-2, 1e-2])
P0 = np.diag([1.0, 1.0])

