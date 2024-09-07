
import PyKDL
import numpy as np
#p - Poll_L_0 - Poll_L_3 Poll_R_2  Poll_R_5
P_in = np.array([
    [0.00986, -0.00212, 0.01490],
    [0.03791, -0.02405, 0.01490],
    [0.09307, -0.03517, 0.01490],
    [0.06008, -0.01316, 0.01490]
])
#put your p out here
P_out = np.array([
    [0.19332, 0.03779, -0.11385],
    [0.18554, 0.07110, -0.12149],
    [0.16620, 0.11089, -0.15391],
    [0.17531, 0.07502, -0.14380]
])

# Step 1: Compute the centroids of the points
C_in = np.mean(P_in, axis=0)
C_out = np.mean(P_out, axis=0)

# Step 2: Centralize the points by subtracting the centroids
P_in_centered = P_in - C_in
P_out_centered = P_out - C_out

# Step 3: Compute the covariance matrix
H = np.dot(P_in_centered.T, P_out_centered)

# Step 4: Perform SVD on the covariance matrix
U, S, Vt = np.linalg.svd(H)

# Step 5: Compute the rotation matrix
R = np.dot(Vt.T, U.T)

# Ensure a proper rotation (det(R) should be 1, not -1)
if np.linalg.det(R) < 0:
    Vt[2, :] *= -1
    R = np.dot(Vt.T, U.T)

# Step 6: Compute the translation vector
t = C_out - np.dot(R, C_in)

R, t

R = PyKDL.Rotation(R[0][0], R[0][1], R[0][2],
                   R[1][0], R[1][1], R[1][2],
                   R[2][0], R[2][1], R[2][2],)

# Convert to RPY
roll, pitch, yaw = R.GetRPY()

# Print the results
print("translation:", t)
print("Roll:", roll)
print("Pitch:", pitch)
print("Yaw:", yaw)