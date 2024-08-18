import numpy as np 

W = 1300
H = 1024
mtx = np.array([1612.360913202219, 0, 605.9392282759331, 0, 1613.069791264788, 445.2610595411001, 0, 0, 1]).reshape(3, 3)

fx = mtx[0, 0]
fy = mtx[1, 1]


# Calculate field of view
fov_fx = 2 * np.arctan(0.5 * H / fx) 
fov_fy = 2 * np.arctan(0.5 * H / fy)

print(f"fov_fx {fov_fx}rad, {np.degrees(fov_fx)} deg")
print(f"fov_fy {fov_fy}rad, {np.degrees(fov_fy)} deg")


print(f"fov of 1.2rad is {np.degrees(1.2)}deg")