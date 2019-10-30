import numpy as np
import cv2
import pose_estimation as pose

npzfile = np.load("pnp_data.npz")
names = npzfile.files
for name in names:
    print(name, npzfile[name].shape)
points_3d = npzfile[names[0]]
shape = points_3d.shape

points_3d = np.reshape(points_3d, (shape[0], 3))
print(points_3d.shape)
points_2d = npzfile[names[1]]
rvec = npzfile[names[2]]
tvec = npzfile[names[3]]

SE = np.zeros((4, 4))
cam_K = np.array(
    [[718.856 ,   0.    , 607.1928],
       [  0.    , 718.856 , 185.2157],
       [  0.    ,   0.    ,   1.    ]]
)
status = pose.solve_pnp(points_3d, points_2d, cam_K, SE)
print("status:", status)
print(SE)
print(rvec)
R, _ = cv2.Rodrigues(rvec)
print(R)
print("t", tvec)
